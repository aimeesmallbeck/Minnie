#!/usr/bin/env python3
"""
ROS2 Node for AimeeCloud Client (Standard ROS2 implementation)

Connects AIMEE robot to AimeeCloud via MQTT.
Subscribes to local ROS2 topics, forwards to cloud.
Receives cloud responses, dispatches to local topics/skills.
"""

import base64
import json
import logging
import os
import ssl
import subprocess
import threading
import time
import uuid
import websocket
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from aimee_msgs.msg import Intent, CloudIntent, ArmCommand, AudioChunk
try:
    from aimee_msgs.srv import CaptureSnapshot
except ImportError:
    CaptureSnapshot = None

logger = logging.getLogger(__name__)

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None
    logger.warning("paho-mqtt not installed. Cloud bridge will not function.")


class AimeeCloudClientNode(Node):
    """Standard ROS2 AimeeCloud Client Node."""

    def __init__(self):
        super().__init__('aimee_cloud_client')

        # Parameters
        self.declare_parameters(namespace='', parameters=[
            ('device_id', 'arduino-uno-q-001'),
            ('broker_host', 'aimeecloud.com'),
            ('broker_port', 443),
            ('ws_endpoint', 'wss://aimeecloud.com/ws/v1'),
            ('api_key', ''),
            ('use_websocket', True),
            ('websocket_path', '/aimeecloud-mqtt'),
            ('user_name', 'Scott'),
            ('user_location', 'home'),
            ('user_language', 'en-US'),
            ('reconnect_interval_sec', 5.0),
            ('ping_interval_sec', 60.0),
            ('session_file', '/home/arduino/.config/aimee_session.json'),
            ('snapshot_resolution', '640x480'),
            ('snapshot_quality', 85),
        ])

        self._device_id = self.get_parameter('device_id').value
        self._broker_host = self.get_parameter('broker_host').value
        self._broker_port = self.get_parameter('broker_port').value
        self._broker_host = self.get_parameter('broker_host').value
        self._broker_port = self.get_parameter('broker_port').value
        self._ws_endpoint = self.get_parameter('ws_endpoint').value
        self._api_key = self.get_parameter('api_key').value or os.environ.get('LEMONFOX_API_KEY', '')
        self._use_websocket = self.get_parameter('use_websocket').value
        self._websocket_path = self.get_parameter('websocket_path').value
        self._user_name = self.get_parameter('user_name').value
        self._user_location = self.get_parameter('user_location').value
        self._user_language = self.get_parameter('user_language').value
        self._reconnect_interval_sec = self.get_parameter('reconnect_interval_sec').value
        self._ping_interval_sec = self.get_parameter('ping_interval_sec').value
        self._session_file = self.get_parameter('session_file').value
        self._snapshot_resolution = self.get_parameter('snapshot_resolution').value
        self._snapshot_quality = self.get_parameter('snapshot_quality').value

        # TODO: Make capabilities dynamic based on active ROS2 nodes
        # e.g., scan node graph for /ugv02_controller -> add "motors",
        #       /arm_controller -> add "arm", /obsbot_camera + /camera -> add "snapshot", etc.
        self._capabilities = {
            "input": ["voice"],
            "output": ["tts", "snapshot"]
        }

        # State
        self._session_id: str = ""
        self._connected = False
        self._mqtt_client = None
        self._state_lock = threading.Lock()

        # Audio WebSocket state
        self._audio_ws = None
        self._audio_ws_connected = False
        self._audio_ws_thread = None

        # Ensure session directory exists
        os.makedirs(os.path.dirname(self._session_file), exist_ok=True)
        self._load_session()

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self._tts_pub = self.create_publisher(String, '/tts/speak', reliable_qos)
        self._play_audio_pub = self.create_publisher(AudioChunk, '/tts/play_audio', reliable_qos)
        self._session_id_pub = self.create_publisher(String, '/cloud/session_id', reliable_qos)
        self._connected_pub = self.create_publisher(Bool, '/cloud/connected', reliable_qos)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self._arm_cmd_pub = self.create_publisher(ArmCommand, '/arm/command', reliable_qos)
        self._game_cmd_pub = self.create_publisher(CloudIntent, '/game/command', reliable_qos)

        # Subscribers
        self.create_subscription(Intent, '/intent/classified', self._on_intent, 10)
        self.create_subscription(CloudIntent, '/cloud/game_move', self._on_cloud_game_move, 10)
        self.create_subscription(String, '/cloud/raw_text', self._on_cloud_raw_text, 10)
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(AudioChunk, '/voice/audio_stream', self._on_audio_chunk, sensor_qos)

        # Service client for snapshot
        if CaptureSnapshot is not None:
            self._snapshot_cli = self.create_client(CaptureSnapshot, '/camera/capture_snapshot')
        else:
            self._snapshot_cli = None

        # Subscriber for manual snapshot uploads from monitor/dashboard
        self.create_subscription(String, '/cloud/snapshot_manual_upload', self._on_manual_snapshot_upload, 10)

        # Subscriber for session clear requests from monitor/dashboard
        self.create_subscription(Bool, '/cloud/clear_session', self._on_clear_session, 10)

        # Timers for reconnect and ping
        self._reconnect_timer = self.create_timer(self._reconnect_interval_sec, self._reconnect_tick)
        self._ping_timer = self.create_timer(self._ping_interval_sec, self._ping_tick)
        self._connected_pub_timer = self.create_timer(5.0, self._publish_connected_status)

        # Initialize MQTT client
        if mqtt is None:
            self.get_logger().error("paho-mqtt is not installed. Cloud bridge cannot start.")
            return

        self._init_mqtt()

        self.get_logger().info(
            f"AimeeCloudClientNode initialized:\n"
            f"  Device ID: {self._device_id}\n"
            f"  Broker: {self._broker_host}:{self._broker_port}\n"
            f"  WebSocket: {self._use_websocket} ({self._websocket_path})\n"
            f"  Session file: {self._session_file}"
        )

    def _iso_timestamp(self) -> str:
        return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

    def _load_session(self):
        if os.path.exists(self._session_file):
            try:
                with open(self._session_file, "r") as f:
                    data = json.load(f)
                    self._session_id = data.get("session_id", "")
                    self.get_logger().info(f"Loaded session: {self._session_id}")
            except Exception as e:
                self.get_logger().warning(f"Failed to load session: {e}")
                self._session_id = ""

    def _save_session(self, session_id: str):
        try:
            with open(self._session_file, "w") as f:
                json.dump({"session_id": session_id, "timestamp": time.time()}, f)
            with self._state_lock:
                self._session_id = session_id
            self._session_id_pub.publish(String(data=session_id))
            self.get_logger().info(f"Saved session: {session_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to save session: {e}")

    def _clear_session(self):
        if os.path.exists(self._session_file):
            try:
                os.remove(self._session_file)
            except Exception:
                pass
        with self._state_lock:
            self._session_id = ""
        self._session_id_pub.publish(String(data=""))
        self.get_logger().info("Session cleared")

    def _init_mqtt(self):
        if self._use_websocket:
            self._mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                transport="websockets"
            )
            self._mqtt_client.ws_set_options(path=self._websocket_path)
            self._mqtt_client.tls_set_context(ssl.create_default_context())
        else:
            self._mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2
            )

        self._mqtt_client.on_connect = self._on_mqtt_connect
        self._mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self._mqtt_client.on_message = self._on_mqtt_message

        # LWT
        disconnect_topic = f"aimeecloud/device/{self._device_id}/connect"
        disconnect_payload = json.dumps({
            "type": "disconnect",
            "device_id": self._device_id,
            "session_id": self._session_id,
            "timestamp": self._iso_timestamp()
        })
        self._mqtt_client.will_set(disconnect_topic, payload=disconnect_payload, qos=1, retain=False)

        # Network loop will be started on first connect attempt in _reconnect_tick
        self._mqtt_loop_started = False
        self.get_logger().info("MQTT client ready")

    # ─────────────────────────────── MQTT Callbacks ───────────────────────────────

    def _on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        try:
            if rc == 0:
                with self._state_lock:
                    self._connected = True
                self.get_logger().info("Connected to AimeeCloud MQTT broker")

                out_topic = f"aimeecloud/device/{self._device_id}/out"
                status_topic = f"aimeecloud/device/{self._device_id}/status"
                system_topic = f"aimeecloud/device/{self._device_id}/system"
                client.subscribe(out_topic, qos=1)
                client.subscribe(status_topic, qos=1)
                client.subscribe(system_topic, qos=1)
                self.get_logger().info(
                    f"Subscribed to {out_topic}, {status_topic}, and {system_topic}"
                )

                self._publish_connect()
                self._connected_pub.publish(Bool(data=True))
            else:
                self.get_logger().error(f"MQTT connect failed with code: {rc}")
                with self._state_lock:
                    self._connected = False
                self._connected_pub.publish(Bool(data=False))
        except Exception as e:
            self.get_logger().error(f"Exception in _on_mqtt_connect: {e}")

    def _on_mqtt_disconnect(self, client, userdata, disconnect_flags, rc, properties=None):
        with self._state_lock:
            self._connected = False
        self.get_logger().warning(f"Disconnected from AimeeCloud MQTT broker (rc={rc})")
        self._connected_pub.publish(Bool(data=False))

    def _on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            topic = msg.topic
            is_pong = payload.get("sub_type") == "pong"
            if is_pong:
                self.get_logger().debug(f"Received MQTT on {topic}: {json.dumps(payload)}")
            else:
                self.get_logger().info(f"Received MQTT on {topic}: {json.dumps(payload)}")

            if topic.endswith("/out"):
                self._handle_out_message(payload)
            elif topic.endswith("/status"):
                self._handle_status_message(payload)
            elif topic.endswith("/system"):
                self._handle_system_message(payload)
        except json.JSONDecodeError as e:
            self.get_logger().warning(f"Failed to decode MQTT message: {e}")
        except Exception as e:
            self.get_logger().error(f"Error handling MQTT message: {e}")

    # ─────────────────────────────── Message Handlers ───────────────────────────────

    def _handle_out_message(self, payload: dict):
        msg_type = payload.get("type")

        if msg_type == "session_init":
            session_id = payload.get("session_id")
            if session_id:
                self._save_session(session_id)
                self.get_logger().info(f"Session initialized: {session_id}")
                # Start audio WebSocket now that we have a session
                self._start_audio_ws()
            return

        if msg_type == "snapshot_request":
            self._handle_snapshot_request(payload)
            return

        if msg_type != "response":
            return

        sub_type = payload.get("sub_type")
        tts = payload.get("tts", "")

        voice = payload.get("voice", {})
        voice_segments = payload.get("voice_segments", [])

        text = payload.get("tts", "") or payload.get("text", "")
        commands = payload.get("commands", [])

        if sub_type == "chat_response":
            self._speak_response(tts, voice, voice_segments)
            for cmd in commands:
                self._execute_command(cmd)
        elif sub_type == "game_update":
            self._speak_response(tts, voice, voice_segments)
            for cmd in commands:
                self._execute_command(cmd)
            self.get_logger().info(f"Game update received with {len(commands)} commands")
        elif sub_type == "robot_command":
            intent = payload.get("intent", "")
            command = payload.get("command", {})
            self._on_robot_command(intent, command, tts, voice, voice_segments)
            for cmd in commands:
                self._execute_command(cmd)
        elif sub_type == "aimee_agent":
            self._speak_response(text, voice, voice_segments)
            for cmd in commands:
                self._execute_command(cmd)
            self.get_logger().info(f"AimeeAgent response handled with {len(commands)} commands")
        elif sub_type == "pong":
            self.get_logger().debug("Received pong from cloud")
        elif sub_type == "error":
            error_code = payload.get("error", "UNKNOWN")
            if error_code == "SESSION_NOT_FOUND":
                self.get_logger().warning("Session not found, clearing and reconnecting")
                self._clear_session()
                self._publish_connect()
            else:
                if tts:
                    self._speak_response(tts, voice, voice_segments)
                for cmd in commands:
                    self._execute_command(cmd)
                self.get_logger().error(f"Cloud error: {error_code}")

    def _handle_status_message(self, payload: dict):
        status = payload.get("status")
        if status == "expired":
            self.get_logger().warning("Session expired by cloud")
            self._clear_session()

    def _handle_system_message(self, payload: dict):
        msg_type = payload.get("type")
        if msg_type != "system_message":
            self.get_logger().debug(f"Ignoring non-system_message on system topic: {msg_type}")
            return

        doc = payload.get("payload", {})
        if doc.get("format") == "markdown" and doc.get("content"):
            msg_id = doc.get("msg_id", "unknown")
            title = doc.get("title", "Untitled Document")
            content = doc.get("content", "")

            docs_dir = os.path.expanduser("~/.config/aimee_docs")
            os.makedirs(docs_dir, exist_ok=True)
            safe_name = "".join(c if c.isalnum() or c in "._-" else "_" for c in msg_id)
            filepath = os.path.join(docs_dir, f"{safe_name}.md")

            try:
                with open(filepath, "w", encoding="utf-8") as f:
                    f.write(f"# {title}\n\n{content}")
                self.get_logger().info(f"Saved system doc: {title} -> {filepath} ({len(content)} bytes)")
            except Exception as e:
                self.get_logger().error(f"Failed to save system doc {msg_id}: {e}")
                return

            self._publish_system_ack(msg_id)
        else:
            self.get_logger().debug(f"Received system message with unsupported format: {doc.get('format')}")

    # ─────────────────────────────── Publishers ───────────────────────────────

    def _publish_connect(self):
        if not self._mqtt_client:
            return
        with self._state_lock:
            if not self._connected:
                return

        topic = f"aimeecloud/device/{self._device_id}/connect"
        payload = {
            "type": "connect",
            "device_id": self._device_id,
            "user_profile": {
                "name": self._user_name,
                "location": self._user_location,
                "language": self._user_language
            },
            "capabilities": self._capabilities,
            "request_session_id": self._session_id,
            "timestamp": self._iso_timestamp()
        }
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        self.get_logger().info(f"Published connect (session: {self._session_id or 'new'})")

    def _publish_connected_status(self):
        """Periodically publish /cloud/connected so late-starting nodes know the state."""
        with self._state_lock:
            is_connected = self._connected
        self._connected_pub.publish(Bool(data=is_connected))

    def _publish_system_ack(self, msg_id: str):
        if not self._mqtt_client:
            return
        with self._state_lock:
            if not self._connected:
                return

        topic = f"aimeecloud/device/{self._device_id}/system/in"
        payload = {
            "type": "ack",
            "session_id": self._session_id,
            "ack_for": msg_id,
            "payload": {"received_at": self._iso_timestamp()},
            "timestamp": self._iso_timestamp()
        }
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        self.get_logger().info(f"Published system ack for {msg_id}")

    def send_intent(self, text: str, intent_dict: dict = None):
        if not self._mqtt_client:
            return
        with self._state_lock:
            if not self._connected:
                self.get_logger().warning("Cannot send intent: not connected")
                return

        topic = f"aimeecloud/device/{self._device_id}/in"
        payload = {
            "type": "intent",
            "device_id": self._device_id,
            "session_id": self._session_id,
            "payload": text,
            "timestamp": self._iso_timestamp()
        }
        if intent_dict:
            payload["intent"] = intent_dict
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        self.get_logger().info(f"Published intent: {text}")

    def send_game_move(self, game: str, move: dict):
        if not self._mqtt_client:
            return
        with self._state_lock:
            if not self._connected:
                self.get_logger().warning("Cannot send game move: not connected")
                return

        topic = f"aimeecloud/device/{self._device_id}/in"
        payload = {
            "type": "game_move",
            "device_id": self._device_id,
            "session_id": self._session_id,
            "game": game,
            "move": move,
            "timestamp": self._iso_timestamp()
        }
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        self.get_logger().info(f"Published game move: {game} {move}")

    def send_ping(self):
        if not self._mqtt_client:
            return
        with self._state_lock:
            if not self._connected:
                return

        topic = f"aimeecloud/device/{self._device_id}/in"
        payload = {
            "type": "ping",
            "device_id": self._device_id,
            "session_id": self._session_id,
            "timestamp": self._iso_timestamp()
        }
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        self.get_logger().debug("Published ping")

    def send_agent_request(self, text: str):
        """Send an AimeeAgent request to AimeeCloud."""
        if not self._mqtt_client:
            return
        with self._state_lock:
            if not self._connected:
                self.get_logger().warning("Cannot send agent request: not connected")
                return

        topic = f"aimeecloud/device/{self._device_id}/in"
        payload = {
            "type": "AimeeAgent",
            "device_id": self._device_id,
            "session_id": self._session_id,
            "payload": text,
            "timestamp": self._iso_timestamp()
        }
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        self.get_logger().info(f"Published AimeeAgent request: {text}")

    # ─────────────────────────────── ROS2 Callbacks ───────────────────────────────

    def _on_intent(self, msg: Intent):
        if msg.skill_name == "AimeeCloud":
            self.send_agent_request(msg.raw_text)

    def _on_cloud_game_move(self, msg: CloudIntent):
        try:
            move = json.loads(msg.move_json) if msg.move_json else {}
        except json.JSONDecodeError:
            move = {}
        self.send_game_move(msg.game_type, move)

    def _on_cloud_raw_text(self, msg: String):
        self.send_agent_request(msg.data)

    def _start_audio_ws(self):
        """Start the audio WebSocket connection in a background thread."""
        if self._audio_ws_thread and self._audio_ws_thread.is_alive():
            return
        
        self.get_logger().info(f"Connecting to audio WebSocket at {self._ws_endpoint}...")
        self._audio_ws_thread = threading.Thread(target=self._audio_ws_run, daemon=True)
        self._audio_ws_thread.start()

    def _audio_ws_run(self):
        """WebSocket event loop."""
        while rclpy.ok():
            try:
                self._audio_ws = websocket.WebSocketApp(
                    self._ws_endpoint,
                    header=[f"x-api-key: {self._api_key}"],
                    on_open=self._on_audio_ws_open,
                    on_message=self._on_audio_ws_message,
                    on_error=self._on_audio_ws_error,
                    on_close=self._on_audio_ws_close
                )
                self._audio_ws.run_forever(ping_interval=30, ping_timeout=10)
            except Exception as e:
                self.get_logger().error(f"Audio WebSocket error: {e}")
            
            # Backoff before reconnecting
            time.sleep(5.0)

    def _on_audio_ws_open(self, ws):
        self.get_logger().info("Audio WebSocket connected.")
        self._audio_ws_connected = True
        
        # Send handshake
        handshake = {
            "type": "session_start",
            "api_key": self._api_key,
            "device_id": self._device_id,
            "session_id": self._session_id,
            "capabilities": {
                "audio_in": { "codec": "pcm16", "sample_rate": 16000 },
                "audio_out": { "codec": "pcm16", "sample_rate": 24000 }
            }
        }
        ws.send(json.dumps(handshake))

    def _on_audio_ws_message(self, ws, message):
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            
            if msg_type == "audio_chunk":
                # Decode and play through TTS node
                audio_b64 = data.get("data")
                if audio_b64:
                    audio_bytes = base64.b64decode(audio_b64)
                    
                    msg = AudioChunk()
                    msg.data = list(audio_bytes)
                    msg.format = data.get("format", "pcm16")
                    msg.sample_rate = data.get("sample_rate", 24000)
                    msg.channels = data.get("channels", 1)
                    msg.timestamp = self.get_clock().now().to_msg()
                    self._play_audio_pub.publish(msg)
                    
                    self.get_logger().debug(f"Received audio chunk seq {data.get('seq')}")
            elif msg_type == "session_ready":
                self.get_logger().info(f"Audio session ready: {data.get('session_id')}")
            elif msg_type == "interrupted":
                self.get_logger().info("Audio stream interrupted by cloud.")
                # Send stop to TTS
                stop_msg = String()
                stop_msg.data = "stop"
                self._tts_pub.publish(stop_msg)
            elif msg_type == "error":
                self.get_logger().error(f"Cloud audio error: {data.get('message')} (code: {data.get('code')})")
        except Exception as e:
            self.get_logger().error(f"Error processing WebSocket message: {e}")

    def _on_audio_ws_error(self, ws, error):
        self.get_logger().error(f"Audio WebSocket error: {error}")

    def _on_audio_ws_close(self, ws, close_status_code, close_msg):
        self.get_logger().info("Audio WebSocket closed.")
        self._audio_ws_connected = False

    def _on_audio_chunk(self, msg: AudioChunk):
        """Handle raw audio chunks from Voice Manager and stream to AimeeCloud via WebSocket."""
        if not self._audio_ws or not self._audio_ws_connected:
            # Try to start it if it should be running
            if self._connected:
                self._start_audio_ws()
            return

        try:
            audio_bytes = bytes(msg.data)
            audio_b64 = base64.b64encode(audio_bytes).decode('utf-8')
            
            payload = {
                "type": "audio_chunk",
                "format": "pcm16",
                "sample_rate": msg.sample_rate,
                "data": audio_b64
            }
            self._audio_ws.send(json.dumps(payload))
        except Exception as e:
            self.get_logger().error(f"Error streaming audio to cloud WebSocket: {e}")

    def _speak_response(self, text: str, voice: dict = None, voice_segments: list = None):
        """Publish TTS text, optionally with voice metadata from AimeeCloud."""
        if voice_segments:
            for segment in voice_segments:
                seg_text = segment.get("text", "")
                seg_voice = segment.get("voice", "")
                if seg_text:
                    speak_text = self._format_tts_text(seg_text, voice_id=seg_voice)
                    self._tts_pub.publish(String(data=speak_text))
                    self.get_logger().info(f"Cloud TTS segment ({seg_voice}): {seg_text[:60]}...")
            return

        if not text:
            return
        speak_text = self._format_tts_text(text, voice=voice)
        self._tts_pub.publish(String(data=speak_text))
        self.get_logger().info(f"Cloud TTS: {text[:60]}...")

    def _format_tts_text(self, text: str, voice: dict = None, voice_id: str = "") -> str:
        """Format TTS text with engine|voice prefix for the TTS node."""
        provider = "lemonfox"
        vid = voice_id
        if not vid and voice:
            vid = voice.get("id", "")
            provider = voice.get("provider", provider)
        if vid:
            return f"{provider}|{vid}:{text}"
        return text

    def _on_manual_snapshot_upload(self, msg: String):
        """Handle a snapshot manually uploaded from the monitor dashboard."""
        try:
            payload = json.loads(msg.data)
            image_b64 = payload.get("image_base64", "")
            request_id = payload.get("request_id", str(uuid.uuid4()))
            session_id = payload.get("session_id", "")
            if not image_b64:
                self.get_logger().warning("Manual snapshot upload received but image_base64 is empty")
                return
            self._publish_snapshot_response(
                session_id, request_id, True,
                "Snapshot uploaded from monitor", image_b64
            )
            self.get_logger().info(f"Manual snapshot uploaded to AimeeCloud: {request_id}")
        except Exception as e:
            self.get_logger().error(f"Manual snapshot upload error: {e}")

    def _on_clear_session(self, msg: Bool):
        """Handle a session clear request from the monitor dashboard."""
        self.get_logger().info("Received session clear request from monitor")
        self._clear_session()
        self._publish_connect()
        self.get_logger().info("Session cleared and reconnect published; next request will use a new session")

    def _handle_snapshot_request(self, payload: dict):
        """Handle snapshot_request from AimeeCloud."""
        session_id = payload.get("session_id", "")
        request_id = payload.get("request_id", "")
        resolution = payload.get("resolution") or self._snapshot_resolution
        quality = payload.get("quality", self._snapshot_quality)

        self.get_logger().info(
            f"Snapshot request received: {request_id} ({resolution}, q={quality})"
        )

        # Ensure service is available
        if not self._snapshot_cli.wait_for_service(timeout_sec=5.0):
            self._publish_snapshot_response(
                session_id, request_id, False,
                "Snapshot service not available", ""
            )
            return

        # Stop usb_camera to free V4L2 device
        usb_cam_stopped = self._stop_usb_camera()
        if not usb_cam_stopped:
            self.get_logger().warning(
                "Failed to stop usb_camera; attempting snapshot anyway"
            )

        try:
            req = CaptureSnapshot.Request()
            req.resolution = resolution
            req.quality = quality
            self.get_logger().info(f"Calling snapshot service with resolution={resolution}, quality={quality}")

            future = self._snapshot_cli.call_async(req)
            # Wait for response with timeout
            timeout_at = time.time() + 12.0
            while not future.done() and time.time() < timeout_at:
                time.sleep(0.1)

            if not future.done():
                self._publish_snapshot_response(
                    session_id, request_id, False,
                    "Snapshot service call timed out", ""
                )
                return

            result = future.result()
            if result.success:
                image_b64 = base64.b64encode(result.image.data).decode("utf-8")
                self._publish_snapshot_response(
                    session_id, request_id, True,
                    result.message, image_b64
                )
                self.get_logger().info(
                    f"Snapshot sent to cloud: {request_id} "
                    f"({len(image_b64)} base64 chars)"
                )
            else:
                self._publish_snapshot_response(
                    session_id, request_id, False,
                    result.message, ""
                )

        except Exception as e:
            self.get_logger().error(f"Snapshot request error: {e}")
            self._publish_snapshot_response(
                session_id, request_id, False,
                f"Snapshot error: {e}", ""
            )

        finally:
            # Always restart usb_camera
            if usb_cam_stopped:
                self._start_usb_camera()

    def _publish_snapshot_response(self, session_id: str, request_id: str,
                                   success: bool, message: str, image_b64: str):
        if not self._mqtt_client:
            return
        with self._state_lock:
            if not self._connected:
                return

        topic = f"aimeecloud/device/{self._device_id}/in"
        payload = {
            "type": "snapshot_response",
            "device_id": self._device_id,
            "session_id": session_id,
            "request_id": request_id,
            "success": success,
            "message": message,
            "format": "jpeg",
            "image_base64": image_b64,
            "timestamp": self._iso_timestamp()
        }
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        self.get_logger().info(f"Published snapshot_response: success={success}")

    def _stop_usb_camera(self) -> bool:
        """Stop usb_camera node to free V4L2 device."""
        try:
            result = subprocess.run(
                ["pkill", "-f", "usb_cam_node_exe"],
                capture_output=True, timeout=5
            )
            time.sleep(1.5)
            self.get_logger().info("Stopped usb_camera node")
            return True
        except Exception as e:
            self.get_logger().warning(f"Failed to stop usb_camera: {e}")
            return False

    def _start_usb_camera(self) -> bool:
        """Start usb_camera node via ros2 run in background."""
        try:
            env = os.environ.copy()
            cmd = (
                "source /opt/ros/humble/setup.bash && "
                "source /workspace/install/setup.bash && "
                "ros2 run usb_cam usb_cam_node_exe --ros-args "
                "-p video_device:=/dev/video2 "
                "-p image_width:=1280 "
                "-p image_height:=720 "
                "-p pixel_format:=mjpeg2rgb "
                "-p io_method:=mmap"
            )
            subprocess.Popen(
                cmd, shell=True, executable="/bin/bash",
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                env=env, start_new_session=True
            )
            self.get_logger().info("Started usb_camera node")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to start usb_camera: {e}")
            return False

    def _on_robot_command(self, intent: str, command: dict, text: str, voice: dict = None, voice_segments: list = None):
        motor = command.get("motor")
        arm = command.get("arm")
        gripper = command.get("gripper")
        duration_ms = command.get("duration_ms", 0)

        if motor:
            twist = Twist()
            if motor == "forward":
                twist.linear.x = 0.5
            elif motor == "backward":
                twist.linear.x = -0.5
            elif motor == "left":
                twist.angular.z = 0.5
            elif motor == "right":
                twist.angular.z = -0.5
            elif motor == "stop":
                pass
            elif motor == "wave":
                pass

            self._cmd_vel_pub.publish(twist)

            if duration_ms > 0 and motor not in ("stop", "wave"):
                def stop_motors():
                    self._cmd_vel_pub.publish(Twist())
                timer = threading.Timer(duration_ms / 1000.0, stop_motors)
                timer.daemon = True
                timer.start()

        if arm or gripper:
            arm_msg = ArmCommand()
            arm_msg.action = arm or gripper
            self._arm_cmd_pub.publish(arm_msg)

        if text:
            self._speak_response(text, voice, voice_segments)

        self.get_logger().info(f"Robot command executed: {intent}")

    def _execute_command(self, cmd: dict):
        """Execute a single command from an AimeeAgent response."""
        cmd_type = cmd.get("type")
        if cmd_type == "motor":
            twist = Twist()
            action = cmd.get("action", "")
            duration_ms = cmd.get("duration_ms", 0)
            if action == "forward":
                twist.linear.x = 0.5
            elif action == "backward":
                twist.linear.x = -0.5
            elif action == "left":
                twist.angular.z = 0.5
            elif action == "right":
                twist.angular.z = -0.5
            elif action == "stop":
                pass
            elif action == "wave":
                pass
            self._cmd_vel_pub.publish(twist)
            if duration_ms > 0 and action not in ("stop", "wave"):
                def stop_motors():
                    self._cmd_vel_pub.publish(Twist())
                timer = threading.Timer(duration_ms / 1000.0, stop_motors)
                timer.daemon = True
                timer.start()
        elif cmd_type == "arm":
            arm_msg = ArmCommand()
            arm_msg.action = cmd.get("action", "")
            self._arm_cmd_pub.publish(arm_msg)
        elif cmd_type == "gripper":
            arm_msg = ArmCommand()
            arm_msg.action = cmd.get("action", "")
            self._arm_cmd_pub.publish(arm_msg)
        elif cmd_type == "snapshot":
            self._execute_snapshot_command(cmd)
        elif cmd_type == "game_move":
            self._execute_game_move_command(cmd)
        else:
            self.get_logger().warning(f"Unknown AimeeAgent command type: {cmd_type}")

    def _execute_snapshot_command(self, cmd: dict):
        """Handle snapshot command from AimeeAgent."""
        if CaptureSnapshot is None:
            self.get_logger().warning("CaptureSnapshot service type not available")
            return
        request_id = str(uuid.uuid4())
        camera = cmd.get("camera", "front")
        purpose = cmd.get("purpose", "analysis")
        self.get_logger().info(f"AimeeAgent snapshot: {camera} ({purpose}), req={request_id}")
        if not self._snapshot_cli or not self._snapshot_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning("Snapshot service not available for AimeeAgent command")
            return
        usb_cam_stopped = self._stop_usb_camera()
        if not usb_cam_stopped:
            self.get_logger().warning("Failed to stop usb_camera; attempting snapshot anyway")
        try:
            req = CaptureSnapshot.Request()
            req.resolution = self._snapshot_resolution
            req.quality = self._snapshot_quality
            self.get_logger().info(f"Calling snapshot service with resolution={self._snapshot_resolution}, quality={self._snapshot_quality}")
            future = self._snapshot_cli.call_async(req)
            timeout_at = time.time() + 12.0
            while not future.done() and time.time() < timeout_at:
                time.sleep(0.1)
            if not future.done():
                self.get_logger().warning("AimeeAgent snapshot command timed out")
                return
            result = future.result()
            if result.success:
                image_b64 = base64.b64encode(result.image.data).decode("utf-8")
                self._publish_snapshot_response(
                    self._session_id, request_id, True,
                    f"Snapshot for {purpose}", image_b64
                )
                self.get_logger().info(f"AimeeAgent snapshot completed: {request_id}")
            else:
                self.get_logger().warning(f"AimeeAgent snapshot failed: {result.message}")
        except Exception as e:
            self.get_logger().error(f"AimeeAgent snapshot error: {e}")
        finally:
            if usb_cam_stopped:
                self._start_usb_camera()

    def _execute_game_move_command(self, cmd: dict):
        """Dispatch a game_move command to the local game handler."""
        game = cmd.get("game", "")
        move = {k: v for k, v in cmd.items() if k not in ("type", "game")}
        cloud_intent = CloudIntent()
        cloud_intent.game_type = game
        try:
            cloud_intent.move_json = json.dumps(move)
        except Exception:
            cloud_intent.move_json = "{}"
        self._game_cmd_pub.publish(cloud_intent)
        self.get_logger().info(f"AimeeAgent game move dispatched: {game} {move}")

    # ─────────────────────────────── Timers ───────────────────────────────

    def _reconnect_tick(self):
        if not self._mqtt_client:
            return
        with self._state_lock:
            is_connected = self._connected
        if not is_connected:
            try:
                self.get_logger().info(
                    f"MQTT connecting to {self._broker_host}:{self._broker_port} ..."
                )
                self._mqtt_client.connect(self._broker_host, self._broker_port, keepalive=60)
                if not self._mqtt_loop_started:
                    self._mqtt_client.loop_start()
                    self._mqtt_loop_started = True
                    self.get_logger().info("MQTT network loop started")
                self.get_logger().info("MQTT connect request queued")
            except Exception as e:
                self.get_logger().warning(f"MQTT reconnect attempt failed: {e}")

    def _ping_tick(self):
        self.send_ping()

    # ─────────────────────────────── Lifecycle ───────────────────────────────

    def destroy_node(self):
        self.get_logger().info("Shutting down AimeeCloudClientNode...")

        if self._reconnect_timer:
            self._reconnect_timer.cancel()
        if self._ping_timer:
            self._ping_timer.cancel()
        if getattr(self, '_connected_pub_timer', None):
            self._connected_pub_timer.cancel()

        if self._mqtt_client:
            try:
                topic = f"aimeecloud/device/{self._device_id}/connect"
                payload = {
                    "type": "disconnect",
                    "device_id": self._device_id,
                    "session_id": self._session_id,
                    "timestamp": self._iso_timestamp()
                }
                self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
            except Exception:
                pass
            self._mqtt_client.loop_stop()
            self._mqtt_client.disconnect()

        super().destroy_node()
        self.get_logger().info("Shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    node = AimeeCloudClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
