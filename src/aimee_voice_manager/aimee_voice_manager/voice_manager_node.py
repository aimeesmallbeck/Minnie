#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
#
# SPDX-License-Identifier: MPL-2.0

"""
ROS2 Node for Voice Manager (Continuous STT)

Standard ROS2 node — migrated from brick pattern.
Publishes Transcription messages on /voice/transcription.
Supports streaming partial results on /voice/partial.
"""

import array
import collections
import io
import json
import logging
import math
import os
import re
import select
import signal
import subprocess
import threading
import time
import wave
from dataclasses import dataclass
from typing import Optional, Callable, Dict, Any, Set, List

import requests
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String, Bool
from aimee_msgs.msg import Transcription, AudioChunk

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class TranscriptionResult:
    """Result from STT engine."""
    text: str
    confidence: float = 1.0
    is_command: bool = False
    engine: str = "vosk"
    language: str = "en"
    wake_word: str = ""
    session_id: str = ""


class VoiceManagerNode(Node):
    """Standard ROS2 Voice Manager node with continuous Vosk STT."""

    def __init__(self):
        super().__init__('voice_manager')

        # ─── Parameters ───
        self.declare_parameters(namespace='', parameters=[
            ('engine', 'vosk'),
            ('model_path', '/home/arduino/vosk-models/vosk-model-small-en-us-0.15'),
            ('sample_rate', 16000),
            ('audio_device', 'default'),
            ('command_timeout', 10.0),
            ('min_command_length', 0.3),
            ('energy_threshold', 45.0),
            ('enabled', True),
            ('publish_partials', True),
            ('debug', False),
            ('whisper_enabled', True),
            ('whisper_api_key', ''),
            ('whisper_api_base_url', 'https://api.openai.com/v1/audio/transcriptions'),
            ('online_topic', '/cloud/connected'),
            ('default_voice', 'sarah'),
            ('stream_to_cloud', False),
        ])

        self._engine = self.get_parameter('engine').value
        self._model_path = self.get_parameter('model_path').value
        self._sample_rate = self.get_parameter('sample_rate').value
        self._audio_device = self.get_parameter('audio_device').value
        self._command_timeout = self.get_parameter('command_timeout').value
        self._min_command_length = self.get_parameter('min_command_length').value
        self._energy_threshold = self.get_parameter('energy_threshold').value
        self._enabled = self.get_parameter('enabled').value
        self._publish_partials = self.get_parameter('publish_partials').value
        self._debug = self.get_parameter('debug').value
        self._whisper_enabled = self.get_parameter('whisper_enabled').value
        self._whisper_api_key = self.get_parameter('whisper_api_key').value or os.environ.get('OPENAI_API_KEY', '')
        self._whisper_api_base_url = self.get_parameter('whisper_api_base_url').value
        self._online_topic = self.get_parameter('online_topic').value
        self._default_voice = self.get_parameter('default_voice').value
        self._stream_to_cloud = self.get_parameter('stream_to_cloud').value

        if self._debug:
            logger.setLevel(logging.DEBUG)

        # ─── QoS ───
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ─── Publishers ───
        self._transcription_pub = self.create_publisher(
            Transcription, '/voice/transcription', reliable_qos
        )
        self._partial_pub = self.create_publisher(
            Transcription, '/voice/partial', sensor_qos
        )
        self._audio_stream_pub = self.create_publisher(
            AudioChunk, '/voice/audio_stream', sensor_qos
        )
        self._status_pub = self.create_publisher(
            String, '/voice/status', reliable_qos
        )

        # ─── Subscribers ───
        self.create_subscription(
            String, '/voice/control', self._on_control_command, 10
        )
        self.create_subscription(
            String, '/tts/speak', self._on_tts_speak,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        )
        self.create_subscription(
            Bool, '/tts/is_speaking', self._on_tts_speaking,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        )
        self.create_subscription(
            Bool, self._online_topic, self._on_online_changed,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        )

        # ─── Parameter callback ───
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        # ─── State ───
        self._vosk_model = None
        self._initialized = False
        self._listening = False
        self._shutdown_event = threading.Event()
        self._listen_thread: Optional[threading.Thread] = None
        self._online = False
        self._online_lock = threading.Lock()

        self.garbage_words: Set[str] = {"huh", "who", "um", "mm", "mhm", "uh", "eh", "hm", "hmm", "e", "a", "i", "o", "u"}

        self._tts_active = False
        self._tts_text_history: List[str] = []
        self._tts_history_lock = threading.Lock()
        self._tts_similarity_threshold = 0.60
        self._tts_energy_boost = 3.5
        self._tts_mute_until = 0.0

        self._last_successful_transcription_time = 0.0
        self._energy_window = collections.deque(maxlen=3)

        self.get_logger().info(
            f"VoiceManagerNode initialized:\n"
            f"  Engine: {self._engine}\n"
            f"  Model: {self._model_path or 'default'}\n"
            f"  Sample rate: {self._sample_rate}Hz\n"
            f"  Audio device: {self._audio_device}\n"
            f"  Energy threshold: {self._energy_threshold}\n"
            f"  Whisper API enabled: {self._whisper_enabled}\n"
            f"  Default voice: {self._default_voice}\n"
            f"  Continuous mode: True"
        )

        # ─── Start background thread ───
        self._listen_thread = threading.Thread(target=self._init_and_listen, daemon=True)
        self._listen_thread.start()

    def _init_and_listen(self):
        """Background thread: initialize Vosk and run listen loop with auto-restart."""
        restart_count = 0
        while not self._shutdown_event.is_set():
            try:
                if not self._initialized:
                    self._load_vosk_model()

                if self._enabled:
                    self._start_listening()
                    # Block until listening stops or errors out
                    while self._listening and not self._shutdown_event.is_set():
                        time.sleep(0.5)
                else:
                    # Just wait until enabled or shutdown
                    while not self._enabled and not self._shutdown_event.is_set():
                        time.sleep(0.5)
                restart_count = 0  # reset on healthy iteration
            except Exception as e:
                self.get_logger().error(f"Listen thread error: {e}")
                self._report_health(False, "listen_thread_error", str(e))

            if self._shutdown_event.is_set():
                break

            restart_count += 1
            # Back off up to 30s so a missing mic/model doesn't spam logs
            delay = min(2.0 * restart_count, 30.0)
            self.get_logger().warning(
                f"Listen loop stopped; restarting in {delay:.0f}s... (attempt {restart_count})"
            )
            self._listening = False
            time.sleep(delay)

    def _ensure_usb_camera_running(self):
        """Start usb_camera node if not already running (required for OBSBOT mic)."""
        try:
            # Check if usb_cam_node_exe is already running
            result = subprocess.run(
                ["pgrep", "-f", "usb_cam_node_exe"],
                capture_output=True, timeout=2
            )
            if result.returncode == 0:
                self.get_logger().debug("usb_camera node is already running")
                return
        except Exception:
            pass

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
            self.get_logger().info("Started usb_camera node for microphone activation")
            time.sleep(3.0)  # Give usb_cam time to open V4L2 and activate the mic interface
        except Exception as e:
            self.get_logger().warning(f"Failed to start usb_camera node: {e}")

    def _load_vosk_model(self):
        """Load the Vosk model."""
        try:
            from vosk import Model
            self.get_logger().info(f"Loading Vosk model from {self._model_path}...")
            if not os.path.exists(self._model_path):
                raise RuntimeError(f"Vosk model not found at {self._model_path}")
            self._vosk_model = Model(self._model_path)
            self._initialized = True
            self.get_logger().info("Vosk model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Vosk: {e}")
            self._report_health(False, "vosk_load_failed", str(e))
            raise

    def _start_listening(self):
        """Start the arecord → Vosk listen loop in this thread."""
        if self._listening:
            return
        # OBSBOT Tiny 2 Lite mic only works when video stream is active
        if 'plughw:2,0' in self._audio_device:
            self._ensure_usb_camera_running()
        self._kill_orphaned_arecord(self._audio_device)
        self._listening = True
        self.get_logger().info("Continuous listening started")
        self._listen_loop()

    def _stop_listening(self):
        """Signal the listen loop to stop."""
        if not self._listening:
            return
        self._shutdown_event.set()
        self._listening = False
        self.get_logger().info("Stopped listening")

    @staticmethod
    def _kill_orphaned_arecord(audio_device: str):
        """Kill any existing arecord processes blocking the audio device."""
        import glob

        card = None
        device = None
        if audio_device.startswith("plughw:") or audio_device.startswith("hw:"):
            try:
                parts = audio_device.split(":", 1)[1].split(",")
                card = int(parts[0])
                device = int(parts[1])
            except Exception:
                pass

        if card is not None and device is not None:
            status_paths = glob.glob(f"/proc/asound/card{card}/pcm{device}c/sub*/status")
            for path in status_paths:
                try:
                    with open(path) as f:
                        for line in f:
                            if line.startswith("owner_pid"):
                                pid_str = line.split(":", 1)[1].strip()
                                pid = int(pid_str)
                                if pid != os.getpid() and os.path.exists(f"/proc/{pid}"):
                                    logger.warning(f"Killing orphaned audio owner PID {pid}")
                                    try:
                                        os.kill(pid, signal.SIGKILL)
                                    except ProcessLookupError:
                                        pass
                                break
                except Exception:
                    pass

        try:
            subprocess.run(
                ["pkill", "-9", "-f", f"arecord.*{audio_device}"],
                capture_output=True, timeout=2
            )
        except Exception:
            pass

        try:
            own_start = os.stat(f"/proc/{os.getpid()}").st_ctime
            for entry in os.listdir("/proc"):
                if not entry.isdigit():
                    continue
                try:
                    pid = int(entry)
                    cmdline = open(f"/proc/{pid}/cmdline").read().replace("\x00", " ")
                    if "arecord" in cmdline:
                        proc_start = os.stat(f"/proc/{pid}").st_ctime
                        if proc_start < own_start:
                            logger.warning(f"Killing stale arecord PID {pid}")
                            os.kill(pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    continue
        except Exception:
            pass

    def _listen_loop(self):
        """Stream audio from arecord to Vosk with auto-recovery on stalls."""
        from vosk import KaldiRecognizer

        cmd = [
            "arecord", "-D", self._audio_device,
            "-f", "S16_LE", "-r", str(self._sample_rate),
            "-c", "1", "-t", "raw"
        ]

        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
            )
            self.get_logger().info(f"arecord started: {' '.join(cmd)}")
        except Exception as e:
            self.get_logger().error(f"Failed to start arecord: {e}")
            self._listening = False
            return

        recognizer = KaldiRecognizer(self._vosk_model, self._sample_rate)

        utterance_has_energy = False
        utterance_buffer = bytearray()
        loop_start = time.time()
        total_bytes = 0
        bytes_since_vosk = 0
        last_health_report = 0.0
        warned_no_data = False
        warned_silence = False
        warned_vosk_stuck = False
        warned_data_stall = False
        last_data_time = time.time()
        zero_energy_streak = 0
        stdout_fd = proc.stdout.fileno()

        try:
            while self._listening and not self._shutdown_event.is_set():
                # Use select with timeout so we can detect arecord stalls
                ready, _, _ = select.select([stdout_fd], [], [], 1.0)
                if not ready:
                    # No data for 1 second — check for stall
                    elapsed = time.time() - loop_start
                    if elapsed > 5.0 and not warned_no_data and total_bytes == 0:
                        warned_no_data = True
                        self.get_logger().error(
                            "Audio capture stall: arecord is running but has produced zero bytes. Restarting..."
                        )
                        self._report_health(
                            False, "audio_stall",
                            "Microphone detected but producing no audio data. Restarting listen loop."
                        )
                        break  # triggers auto-restart
                    if proc.poll() is not None:
                        self.get_logger().error(
                            f"arecord exited with code {proc.returncode}. Restarting..."
                        )
                        self._report_health(
                            False, "arecord_died",
                            f"arecord exited with code {proc.returncode}"
                        )
                        break
                    continue

                data = os.read(stdout_fd, 4000)
                if not data:
                    if proc.poll() is not None:
                        self.get_logger().error(
                            f"arecord exited with code {proc.returncode}. Restarting..."
                        )
                        self._report_health(
                            False, "arecord_died",
                            f"arecord exited with code {proc.returncode}"
                        )
                        break
                    time.sleep(0.01)
                    continue

                total_bytes += len(data)
                last_data_time = time.time()
                elapsed = time.time() - loop_start
                energy = self._audio_energy(data)

                if elapsed > 5.0 and not warned_no_data and total_bytes == 0:
                    warned_no_data = True
                    self.get_logger().error(
                        "Audio capture stall: arecord is running but has produced zero bytes"
                    )
                    self._report_health(
                        False, "audio_stall",
                        "Microphone detected but producing no audio data. Try power-cycling the USB device."
                    )

                # Detect arecord alive but not producing data (e.g. USB reconnect)
                stall_elapsed = time.time() - last_data_time
                if not warned_data_stall and total_bytes > 0 and stall_elapsed > 15.0:
                    warned_data_stall = True
                    self.get_logger().error(
                        f"Audio capture data stall: no audio for {stall_elapsed:.0f}s. Restarting..."
                    )
                    self._report_health(
                        False, "data_stall",
                        "Audio stream stalled mid-session. Likely USB device reconnect. Restarting listen loop."
                    )
                    break

                if energy == 0.0:
                    zero_energy_streak += 1
                    if not warned_silence and zero_energy_streak >= 125:
                        warned_silence = True
                        self.get_logger().error(
                            "Audio capture silence flood: microphone is streaming zero-filled buffers. Restarting..."
                        )
                        self._report_health(
                            False, "silence_flood",
                            "Microphone is connected but streaming silence. Restarting listen loop."
                        )
                        break  # triggers auto-restart
                else:
                    zero_energy_streak = 0

                # Publish raw chunk if cloud streaming is enabled
                if self._stream_to_cloud and not self._tts_active:
                    self._publish_audio_chunk(data)

                # Hard gate: discard audio while TTS is active
                if self._tts_active:
                    utterance_buffer.clear()
                    utterance_has_energy = False
                    self._energy_window.clear()
                    continue

                # Post-TTS grace period
                if time.time() < self._tts_mute_until:
                    continue

                bytes_since_vosk += len(data)
                utterance_buffer.extend(data)
                self._energy_window.append(energy)
                smoothed_energy = sum(self._energy_window) / len(self._energy_window)

                if elapsed > 30.0 and not warned_vosk_stuck and bytes_since_vosk > 480000:
                    warned_vosk_stuck = True
                    self.get_logger().warning(
                        "Vosk has not accepted an utterance in 30+ seconds; microphone may be muted or ambient volume too low"
                    )
                    self._report_health(
                        False, "vosk_stall",
                        "Audio is flowing but Vosk has not recognized any speech in 30s. Check mic volume or mute switch."
                    )

                if total_bytes > 0 and energy > 0.0 and elapsed - last_health_report > 10.0:
                    last_health_report = elapsed
                    self._report_health(True, "", "Audio capture is healthy")

                effective_threshold = self._effective_energy_threshold()
                if smoothed_energy >= effective_threshold:
                    utterance_has_energy = True

                if recognizer.AcceptWaveform(data):
                    bytes_since_vosk = 0
                    result_dict = json.loads(recognizer.Result())
                    vosk_text = result_dict.get("text", "").strip()

                    final_text = vosk_text
                    engine_used = self._engine

                    with self._online_lock:
                        should_use_whisper = self._online and self._whisper_enabled and self._whisper_api_key

                    if should_use_whisper and self._engine == 'whisper_api' and utterance_has_energy and len(utterance_buffer) > 0:
                        whisper_text = self._transcribe_whisper_api(bytes(utterance_buffer))
                        if whisper_text:
                            final_text = whisper_text
                            engine_used = "whisper_api"

                    if final_text and len(final_text) >= self._min_command_length:
                        if self._is_garbage(final_text):
                            self.get_logger().debug(f"Dropped garbage: {final_text}")
                        elif self._is_tts_echo(final_text):
                            self.get_logger().debug(f"Dropped TTS echo: {final_text}")
                        else:
                            self.get_logger().info(f"Transcription ({engine_used}): {final_text}")
                            self._last_successful_transcription_time = time.time()
                            result_obj = TranscriptionResult(
                                text=final_text,
                                confidence=1.0,
                                is_command=True,
                                engine=engine_used,
                                wake_word="",
                                session_id=""
                            )
                            self._publish_transcription(result_obj)

                    utterance_buffer = bytearray()
                    utterance_has_energy = False

                elif self._publish_partials:
                    partial_dict = json.loads(recognizer.PartialResult())
                    partial_text = partial_dict.get("partial", "").strip()
                    if partial_text and self._is_garbage(partial_text):
                        partial_text = ""
                    if partial_text:
                        partial_result = TranscriptionResult(
                            text=partial_text,
                            confidence=0.5,
                            is_command=False,
                            engine=self._engine,
                            wake_word="",
                            session_id=""
                        )
                        self._publish_partial(partial_result)

        except Exception as e:
            self.get_logger().error(f"Listen loop error: {e}")
        finally:
            try:
                proc.terminate()
                proc.wait(timeout=2)
            except Exception:
                pass
            self._listening = False
            self.get_logger().info("Listen loop ended")

    # ─────────────────────────────── Audio & Transcription Helpers ───────────────────────────────

    @staticmethod
    def _audio_energy(data: bytes) -> float:
        """Compute RMS energy of a S16_LE raw audio chunk."""
        if len(data) < 2:
            return 0.0
        try:
            samples = array.array('h', data)
            if len(samples) == 0:
                return 0.0
            sum_squares = sum(s * s for s in samples)
            return math.sqrt(sum_squares / len(samples))
        except Exception:
            return 0.0

    def _is_garbage(self, text: str) -> bool:
        """Check if transcription is just noise garbage."""
        clean = text.lower().strip().rstrip('.?!')
        if not clean or len(clean) < 2:
            return True
        if re.match(r'^(.)\1+$', clean):
            return True
        return clean in self.garbage_words

    def _effective_energy_threshold(self) -> float:
        """Return energy threshold adjusted for TTS playback."""
        if self._tts_active:
            return self._energy_threshold * self._tts_energy_boost
        return self._energy_threshold

    # ─────────────────────────────── TTS Echo Suppression ───────────────────────────────

    def set_tts_active(self, active: bool):
        """Set TTS speaking state for echo suppression."""
        if active and not self._tts_active:
            self.get_logger().debug("TTS started — echo suppression active")
        if not active and self._tts_active:
            self._tts_mute_until = time.time() + 2.0
            self.get_logger().debug(
                f"TTS ended — post-TTS mute window until {self._tts_mute_until:.1f}"
            )
        self._tts_active = active
        self.get_logger().debug(f"TTS active set to {active}")

    def set_tts_text(self, text: str):
        """Record TTS text being spoken for echo correlation."""
        normalized = self._normalize_for_echo(text)
        with self._tts_history_lock:
            self._tts_text_history.append(normalized)
            while len(self._tts_text_history) > 5:
                self._tts_text_history.pop(0)
        self.get_logger().debug(f"Recorded TTS text: {text} -> {normalized}")

    @staticmethod
    def _normalize_for_echo(text: str) -> str:
        """Normalize text for echo comparison."""
        clean = text.lower().strip()
        clean = re.sub(r"[^a-z0-9\s]", "", clean)
        clean = re.sub(r"\s+", " ", clean).strip()
        clean = clean.replace("amy", "aimee")
        return clean

    @staticmethod
    def _levenshtein_ratio(s1: str, s2: str) -> float:
        """Return similarity ratio (0.0-1.0) using Levenshtein distance."""
        m, n = len(s1), len(s2)
        if m == 0 and n == 0:
            return 1.0
        if m == 0 or n == 0:
            return 0.0
        previous = list(range(n + 1))
        for i in range(m):
            current = [i + 1]
            for j in range(n):
                insertions = previous[j + 1] + 1
                deletions = current[j] + 1
                substitutions = previous[j] + (s1[i] != s2[j])
                current.append(min(insertions, deletions, substitutions))
            previous = current
        distance = previous[-1]
        return 1.0 - (distance / max(m, n))

    def _is_tts_echo(self, text: str) -> bool:
        """Check if transcription is the robot hearing its own TTS."""
        clean = self._normalize_for_echo(text)

        if time.time() < self._tts_mute_until:
            self.get_logger().debug(f"Dropped transcription in post-TTS mute window: {text}")
            return True

        if not self._tts_active and not self._tts_text_history:
            return False

        with self._tts_history_lock:
            for tts_text in self._tts_text_history:
                ratio = self._levenshtein_ratio(clean, tts_text)
                if ratio >= self._tts_similarity_threshold:
                    self.get_logger().info(f"Dropped TTS echo (similarity {ratio:.2f}): {text}")
                    return True
        return False

    # ─────────────────────────────── Whisper API ───────────────────────────────

    def _transcribe_whisper_api(self, audio_bytes: bytes) -> Optional[str]:
        """Send raw PCM audio to Whisper API and return transcription text."""
        try:
            wav_buffer = io.BytesIO()
            with wave.open(wav_buffer, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(self._sample_rate)
                wf.writeframes(audio_bytes)
            wav_buffer.seek(0)

            url = self._whisper_api_base_url
            headers = {"Authorization": f"Bearer {self._whisper_api_key}"}
            files = {"file": ("utterance.wav", wav_buffer, "audio/wav")}
            data = {"model": "whisper-1", "language": "en", "response_format": "json"}

            resp = requests.post(url, headers=headers, files=files, data=data, timeout=10)
            resp.raise_for_status()
            text = resp.json().get("text", "").strip()
            self.get_logger().info(f"Whisper API transcription: '{text[:60]}...'")
            return text
        except Exception as e:
            self.get_logger().warning(f"Whisper API transcription failed: {e}")
            return None

    # ─────────────────────────────── ROS2 Callbacks ───────────────────────────────

    def _publish_transcription(self, result: TranscriptionResult):
        if self._stream_to_cloud:
            self.get_logger().info(f"Streaming mode active: suppressing local transcription publishing for '{result.text[:30]}...'")
            return
            
        msg = Transcription()
        msg.text = result.text
        msg.confidence = result.confidence
        msg.source = result.engine
        msg.is_command = result.is_command
        msg.is_partial = False
        msg.wake_word_detected = False
        msg.wake_word = ""
        msg.timestamp = self.get_clock().now().to_msg()
        msg.session_id = result.session_id
        self._transcription_pub.publish(msg)
        self.get_logger().info(
            f"Published transcription: '{result.text[:50]}...' "
            f"(confidence: {result.confidence:.2f})"
        )

    def _publish_partial(self, result: TranscriptionResult):
        if self._stream_to_cloud:
            return
            
        msg = Transcription()
        msg.text = result.text
        msg.confidence = result.confidence
        msg.source = result.engine
        msg.is_command = result.is_command
        msg.is_partial = True
        msg.wake_word_detected = False
        msg.wake_word = ""
        msg.timestamp = self.get_clock().now().to_msg()
        msg.session_id = result.session_id
        self._partial_pub.publish(msg)

    def _publish_audio_chunk(self, data: bytes):
        """Publish raw audio chunk for cloud streaming."""
        msg = AudioChunk()
        msg.data = list(data)
        msg.format = "pcm_s16le"
        msg.sample_rate = self._sample_rate
        msg.channels = 1
        msg.timestamp = self.get_clock().now().to_msg()
        msg.session_id = "" # TODO: Link to session if available
        self._audio_stream_pub.publish(msg)

    def _on_online_changed(self, msg: Bool):
        with self._online_lock:
            if self._online == msg.data:
                return
            self._online = msg.data
        self.get_logger().info(
            f"Online state changed: {msg.data} "
            f"(Whisper {'active' if msg.data else 'fallback to Vosk'})"
        )

    def _on_tts_speak(self, msg: String):
        self.set_tts_text(msg.data)

    def _on_tts_speaking(self, msg: Bool):
        self.set_tts_active(msg.data)

    def _on_control_command(self, msg: String):
        command = msg.data.lower().strip()
        self.get_logger().info(f"Received control command: {command}")

        if command == 'start':
            self._enabled = True
            if self._initialized and not self._listening:
                # Wake up the init thread if it's waiting
                pass  # the init_and_listen loop will pick this up
        elif command == 'stop':
            self._enabled = False
            self._listening = False
            self.get_logger().info("Listening stopped")
        elif command == 'status':
            status = {
                "initialized": self._initialized,
                "engine": self._engine,
                "listening": self._listening,
                "model_path": self._model_path,
            }
            self.get_logger().info(f"Status: {status}")
        else:
            self.get_logger().warning(f"Unknown command: {command}")

    def _on_parameters_changed(self, params):
        results = []
        for param in params:
            if param.name == 'enabled':
                self._enabled = param.value
                if not self._enabled:
                    self._listening = False
                self.get_logger().info(f"Updated enabled: {param.value}")
                results.append(SetParametersResult(successful=True))
            elif param.name == 'publish_partials':
                self._publish_partials = param.value
                self.get_logger().info(f"Updated publish_partials: {param.value}")
                results.append(SetParametersResult(successful=True))
            elif param.name == 'min_command_length':
                self._min_command_length = param.value
                self.get_logger().info(f"Updated min_command_length: {param.value}")
                results.append(SetParametersResult(successful=True))
            elif param.name == 'whisper_api_key':
                self._whisper_api_key = param.value
                self.get_logger().info("Updated whisper_api_key")
                results.append(SetParametersResult(successful=True))
            elif param.name == 'whisper_api_base_url':
                self._whisper_api_base_url = param.value
                self.get_logger().info(f"Updated whisper_api_base_url: {param.value}")
                results.append(SetParametersResult(successful=True))
            else:
                results.append(SetParametersResult(
                    successful=False,
                    reason=f"Parameter {param.name} is read-only after initialization"
                ))
        return results

    def _report_health(self, healthy: bool, issue: str = "", message: str = ""):
        status_msg = String()
        if not healthy:
            status_msg.data = f"ERROR: {issue} — {message}"
            self.get_logger().error(status_msg.data)
        else:
            status_msg.data = f"OK: {message}"
            self.get_logger().debug(status_msg.data)
        self._status_pub.publish(status_msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down VoiceManagerNode...")
        self._shutdown_event.set()
        self._listening = False
        if self._listen_thread and self._listen_thread.is_alive():
            self._listen_thread.join(timeout=3.0)
        super().destroy_node()
        self.get_logger().info("Shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
