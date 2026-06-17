#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
#
# SPDX-License-Identifier: MPL-2.0

"""
CloudBridge Brick - MQTT client for AimeeCloud communication

Handles:
- MQTT connection with auto-reconnect
- Session persistence
- Protocol v1.0 message formatting
- Inbound/outbound message routing
"""

import asyncio
import json
import logging
import os
import time
from typing import Callable, Dict, Optional, Any

from arduino.app_utils import brick

logger = logging.getLogger(__name__)

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None
    logger.warning("paho-mqtt not installed. Cloud bridge will not function.")


class CloudBridgeBrickError(Exception):
    """Custom exception for CloudBridgeBrick errors."""
    pass


@brick
class CloudBridgeBrick:
    """
    MQTT bridge to AimeeCloud.
    
    Implements the AimeeCloud Robot Communication Protocol v1.0.
    """
    
    def __init__(
        self,
        device_id: str = "arduino-uno-q-001",
        robot_name: str = "Minnie",
        robot_personality: str = "Adorable Brat",
        gemini_voice: str = "Fenrir",
        robot_config: Optional[Dict[str, Any]] = None,
        session_context: Optional[Dict[str, Any]] = None,
        broker_host: str = "aimeecloud.com",
        broker_port: int = 1883,
        use_websocket: bool = False,
        websocket_path: str = "/aimeecloud-mqtt",
        user_name: str = "Scott",
        user_location: str = "home",
        user_language: str = "en-US",
        capabilities: Optional[Dict[str, Any]] = None,
        reconnect_interval_sec: float = 5.0,
        ping_interval_sec: float = 60.0,
        session_file: str = "/home/arduino/.config/aimee_session.json",
        on_chat_response: Optional[Callable[[str], None]] = None,
        on_game_update: Optional[Callable[[Dict[str, Any], str], None]] = None,
        on_robot_command: Optional[Callable[[str, Dict[str, Any], str], None]] = None,
        on_error: Optional[Callable[[str, str], None]] = None,
        on_connected: Optional[Callable[[bool], None]] = None,
        on_session_id: Optional[Callable[[str], None]] = None,
    ):
        self.device_id = device_id
        self.robot_name = robot_name
        self.robot_personality = robot_personality
        self.gemini_voice = gemini_voice
        self.robot_config = robot_config or {
            "has_motors": True,
            "has_arm": False,
            "has_gripper": False,
            "has_camera": True,
            "has_expressions": True,
            "expression_types": ["happy", "sad", "surprised", "greeting", "celebration"]
        }
        self.session_context = session_context or {
            "ram_mb": 512,
            "storage_gb": 32,
            "cpu": "Arduino UNO Q",
            "battery": "18650 Li-ion 2600mAh",
            "manufacturer": "Arduino",
            "model": "UNO R4 WiFi"
        }
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.use_websocket = use_websocket
        self.websocket_path = websocket_path
        self.user_name = user_name
        self.user_location = user_location
        self.user_language = user_language
        self.capabilities = capabilities or {
            "input": ["voice", "text"],
            "output": ["tts", "display", "motors", "led"]
        }
        self.reconnect_interval_sec = reconnect_interval_sec
        self.ping_interval_sec = ping_interval_sec
        self.session_file = session_file
        
        # Callbacks
        self._on_chat_response = on_chat_response
        self._on_game_update = on_game_update
        self._on_robot_command = on_robot_command
        self._on_error = on_error
        self._on_connected = on_connected
        self._on_session_id = on_session_id
        
        # State
        self._initialized = False
        self._session_id: Optional[str] = None
        self._connected = False
        self._mqtt_client: Optional[mqtt.Client] = None
        self._reconnect_task: Optional[asyncio.Task] = None
        self._ping_task: Optional[asyncio.Task] = None
        self._shutdown_event = asyncio.Event()
        
        # Ensure session directory exists
        os.makedirs(os.path.dirname(self.session_file), exist_ok=True)
        
        logger.info(f"CloudBridgeBrick initialized for device: {device_id}")
    
    async def initialize(self) -> "CloudBridgeBrick":
        """Initialize the cloud bridge."""
        if self._initialized:
            return self
        
        if mqtt is None:
            raise CloudBridgeBrickError("paho-mqtt is not installed")
        
        self._loop = asyncio.get_event_loop()
        self._load_session()
        if self.use_websocket:
            import ssl
            self._mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                transport="websockets"
            )
            self._mqtt_client.ws_set_options(path=self.websocket_path)
            self._mqtt_client.tls_set_context(ssl.create_default_context())
        else:
            self._mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2
            )
        self._mqtt_client.on_connect = self._on_connect
        self._mqtt_client.on_disconnect = self._on_disconnect
        self._mqtt_client.on_message = self._on_message
        
        # Set LWT for unexpected disconnects
        disconnect_topic = f"aimeecloud/device/{self.device_id}/connect"
        disconnect_payload = json.dumps({
            "type": "disconnect",
            "device_id": self.device_id,
            "session_id": self._session_id or "",
            "timestamp": self._iso_timestamp()
        })
        self._mqtt_client.will_set(
            disconnect_topic,
            payload=disconnect_payload,
            qos=1,
            retain=False
        )
        
        self._initialized = True
        self._reconnect_task = asyncio.create_task(self._reconnect_loop())
        self._ping_task = asyncio.create_task(self._ping_loop())
        
        logger.info("CloudBridgeBrick initialized successfully")
        return self
    
    def _iso_timestamp(self) -> str:
        """Return ISO 8601 UTC timestamp."""
        from datetime import datetime, timezone
        return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    
    def _load_session(self):
        """Load stored session ID from disk."""
        if os.path.exists(self.session_file):
            try:
                with open(self.session_file, "r") as f:
                    data = json.load(f)
                    self._session_id = data.get("session_id")
                    logger.info(f"Loaded session: {self._session_id}")
            except Exception as e:
                logger.warning(f"Failed to load session: {e}")
                self._session_id = None
    
    def _save_session(self, session_id: str):
        """Save session ID to disk."""
        try:
            with open(self.session_file, "w") as f:
                json.dump({
                    "session_id": session_id,
                    "timestamp": time.time()
                }, f)
            self._session_id = session_id
            if self._on_session_id:
                self._on_session_id(session_id)
            logger.info(f"Saved session: {session_id}")
        except Exception as e:
            logger.error(f"Failed to save session: {e}")
    
    def _clear_session(self):
        """Clear stored session ID."""
        if os.path.exists(self.session_file):
            try:
                os.remove(self.session_file)
            except Exception:
                pass
        self._session_id = None
        if self._on_session_id:
            self._on_session_id("")
        logger.info("Session cleared")
    
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        """MQTT connect callback."""
        if rc == 0:
            self._connected = True
            logger.info("Connected to AimeeCloud MQTT broker")
            
            # Subscribe to out, status, and system BEFORE publishing connect
            out_topic = f"aimeecloud/device/{self.device_id}/out"
            status_topic = f"aimeecloud/device/{self.device_id}/status"
            system_topic = f"aimeecloud/device/{self.device_id}/system"
            client.subscribe(out_topic, qos=1)
            client.subscribe(status_topic, qos=1)
            client.subscribe(system_topic, qos=1)
            logger.info(f"Subscribed to {out_topic}, {status_topic}, and {system_topic}")
            
            # Publish connect message
            self._publish_connect()
            
            if self._on_connected:
                self._on_connected(True)
        else:
            logger.error(f"MQTT connect failed with code: {rc}")
            self._connected = False
            if self._on_connected:
                self._on_connected(False)
    
    def _on_disconnect(self, client, userdata, disconnect_flags, rc, properties=None):
        """MQTT disconnect callback."""
        self._connected = False
        logger.warning(f"Disconnected from AimeeCloud MQTT broker (rc={rc})")
        if self._on_connected:
            self._on_connected(False)
    
    def _on_message(self, client, userdata, msg):
        """MQTT message callback."""
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            topic = msg.topic
            logger.debug(f"Received on {topic}: {json.dumps(payload)[:200]}")
            
            if self._loop and self._loop.is_running():
                if topic.endswith("/out"):
                    asyncio.run_coroutine_threadsafe(
                        self._handle_out_message(payload), self._loop
                    )
                elif topic.endswith("/status"):
                    asyncio.run_coroutine_threadsafe(
                        self._handle_status_message(payload), self._loop
                    )
                elif topic.endswith("/system"):
                    asyncio.run_coroutine_threadsafe(
                        self._handle_system_message(payload), self._loop
                    )
        except json.JSONDecodeError as e:
            logger.warning(f"Failed to decode MQTT message: {e}")
        except Exception as e:
            logger.error(f"Error handling MQTT message: {e}")
    
    async def _handle_out_message(self, payload: Dict[str, Any]):
        """Handle messages from the cloud 'out' topic."""
        msg_type = payload.get("type")
        
        if msg_type == "session_init":
            session_id = payload.get("session_id")
            if session_id:
                self._save_session(session_id)
                logger.info(f"Session initialized: {session_id}")
            return
        
        if msg_type != "response":
            return
        
        sub_type = payload.get("sub_type")
        tts = payload.get("tts", "")
        
        if sub_type == "chat_response":
            if self._on_chat_response:
                self._on_chat_response(tts)
        
        elif sub_type == "game_update":
            state = payload.get("state", {})
            if self._on_game_update:
                self._on_game_update(state, tts)
            elif self._on_chat_response:
                self._on_chat_response(tts)
        
        elif sub_type == "robot_command":
            intent = payload.get("intent", "")
            command = payload.get("command", {})
            if self._on_robot_command:
                self._on_robot_command(intent, command, tts)
            elif self._on_chat_response:
                self._on_chat_response(tts)
        
        elif sub_type == "interstitial":
            logger.debug(f"Received interstitial from cloud: {tts[:60]}...")
        
        elif sub_type == "pong":
            logger.debug("Received pong from cloud")
        
        elif sub_type == "error":
            error_code = payload.get("error", "UNKNOWN")
            if error_code == "SESSION_NOT_FOUND":
                logger.warning("Session not found, clearing and reconnecting")
                self._clear_session()
                self._publish_connect()
            else:
                if self._on_error:
                    self._on_error(error_code, tts)
                elif self._on_chat_response:
                    self._on_chat_response(tts)
    
    async def _handle_status_message(self, payload: Dict[str, Any]):
        """Handle messages from the cloud 'status' topic."""
        status = payload.get("status")
        if status == "expired":
            logger.warning("Session expired by cloud")
            self._clear_session()
    
    async def _handle_system_message(self, payload: Dict[str, Any]):
        """Handle system messages from the cloud (docs, config updates, etc.)."""
        msg_type = payload.get("type")
        if msg_type != "system_message":
            logger.debug(f"Ignoring non-system_message on system topic: {msg_type}")
            return
        
        doc = payload.get("payload", {})
        if doc.get("format") == "markdown" and doc.get("content"):
            msg_id = doc.get("msg_id", "unknown")
            title = doc.get("title", "Untitled Document")
            content = doc.get("content", "")
            
            # Save to filesystem
            docs_dir = os.path.expanduser("~/.config/aimee_docs")
            os.makedirs(docs_dir, exist_ok=True)
            safe_name = "".join(c if c.isalnum() or c in "._-" else "_" for c in msg_id)
            filepath = os.path.join(docs_dir, f"{safe_name}.md")
            
            try:
                with open(filepath, "w", encoding="utf-8") as f:
                    f.write(f"# {title}\n\n{content}")
                logger.info(f"Saved system doc: {title} -> {filepath} ({len(content)} bytes)")
            except Exception as e:
                logger.error(f"Failed to save system doc {msg_id}: {e}")
                return
            
            # Publish ack
            self._publish_system_ack(msg_id)
        else:
            logger.debug(f"Received system message with unsupported format: {doc.get('format')}")
    
    def _publish_system_ack(self, msg_id: str):
        """Publish ack for a received system message."""
        if not self._mqtt_client or not self._connected:
            return
        
        topic = f"aimeecloud/device/{self.device_id}/system/in"
        payload = {
            "type": "ack",
            "session_id": self._session_id or "",
            "ack_for": msg_id,
            "payload": {
                "received_at": self._iso_timestamp()
            },
            "timestamp": self._iso_timestamp()
        }
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        logger.info(f"Published system ack for {msg_id}")
    
    def _publish_connect(self):
        """Publish connect message to cloud."""
        if not self._mqtt_client or not self._connected:
            return
        
        topic = f"aimeecloud/device/{self.device_id}/connect"
        payload = {
            "type": "connect",
            "device_id": self.device_id,
            "robot_name": self.robot_name,
            "robot_personality": self.robot_personality,
            "gemini_voice": self.gemini_voice,
            "user_profile": {
                "name": self.user_name,
                "location": self.user_location,
                "language": self.user_language
            },
            "capabilities": self.capabilities,
            "robot_config": self.robot_config,
            "session_context": self.session_context,
            "request_session_id": self._session_id,
            "timestamp": self._iso_timestamp()
        }
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        logger.info(f"Published connect (session: {self._session_id or 'new'})")
    
    def send_intent(self, text: str, intent_dict: Optional[Dict[str, Any]] = None):
        """Publish intent message to cloud."""
        if not self._mqtt_client or not self._connected:
            logger.warning("Cannot send intent: not connected")
            return
        
        topic = f"aimeecloud/device/{self.device_id}/in"
        payload = {
            "type": "intent",
            "device_id": self.device_id,
            "session_id": self._session_id or "",
            "payload": text,
            "timestamp": self._iso_timestamp()
        }
        if intent_dict:
            payload["intent"] = intent_dict
        
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        logger.info(f"Published intent: {text[:50]}")
    
    def send_game_move(self, game: str, move: Dict[str, Any]):
        """Publish game move message to cloud."""
        if not self._mqtt_client or not self._connected:
            logger.warning("Cannot send game move: not connected")
            return
        
        topic = f"aimeecloud/device/{self.device_id}/in"
        payload = {
            "type": "game_move",
            "device_id": self.device_id,
            "session_id": self._session_id or "",
            "game": game,
            "move": move,
            "timestamp": self._iso_timestamp()
        }
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        logger.info(f"Published game move: {game} {move}")
    
    def send_ping(self):
        """Publish ping message to cloud."""
        if not self._mqtt_client or not self._connected:
            return
        
        topic = f"aimeecloud/device/{self.device_id}/in"
        payload = {
            "type": "ping",
            "device_id": self.device_id,
            "session_id": self._session_id or "",
            "timestamp": self._iso_timestamp()
        }
        self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
        logger.debug("Published ping")
    
    async def _reconnect_loop(self):
        """Background task: maintain MQTT connection."""
        while not self._shutdown_event.is_set():
            try:
                if self._mqtt_client and not self._connected:
                    logger.info(f"Connecting to {self.broker_host}:{self.broker_port}...")
                    try:
                        self._mqtt_client.connect(self.broker_host, self.broker_port, keepalive=60)
                        self._mqtt_client.loop_start()
                    except Exception as e:
                        logger.warning(f"Connection attempt failed: {e}")
                
                await asyncio.wait_for(
                    self._shutdown_event.wait(),
                    timeout=self.reconnect_interval_sec
                )
            except asyncio.TimeoutError:
                pass
            except Exception as e:
                logger.error(f"Reconnect loop error: {e}")
    
    async def _ping_loop(self):
        """Background task: send periodic pings."""
        while not self._shutdown_event.is_set():
            try:
                await asyncio.wait_for(
                    self._shutdown_event.wait(),
                    timeout=self.ping_interval_sec
                )
            except asyncio.TimeoutError:
                if self._connected:
                    self.send_ping()
    
    async def shutdown(self):
        """Clean shutdown."""
        logger.info("Shutting down CloudBridgeBrick...")
        self._shutdown_event.set()
        
        # Publish graceful disconnect
        if self._mqtt_client and self._connected:
            topic = f"aimeecloud/device/{self.device_id}/connect"
            payload = {
                "type": "disconnect",
                "device_id": self.device_id,
                "session_id": self._session_id or "",
                "timestamp": self._iso_timestamp()
            }
            try:
                self._mqtt_client.publish(topic, json.dumps(payload), qos=1)
            except Exception:
                pass
            self._mqtt_client.loop_stop()
            self._mqtt_client.disconnect()
        
        # Cancel background tasks
        if self._reconnect_task:
            self._reconnect_task.cancel()
            try:
                await self._reconnect_task
            except asyncio.CancelledError:
                pass
        
        if self._ping_task:
            self._ping_task.cancel()
            try:
                await self._ping_task
            except asyncio.CancelledError:
                pass
        
        self._initialized = False
        self._connected = False
        logger.info("CloudBridgeBrick shutdown complete")
