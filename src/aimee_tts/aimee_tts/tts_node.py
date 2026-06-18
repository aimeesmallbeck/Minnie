#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
#
# SPDX-License-Identifier: MPL-2.0

"""
ROS2 Node for Text-to-Speech (standard node, no brick abstraction)

Supports:
- kokoro  (primary, offline, high quality, multiple voices)
- gtts    (cloud fallback)

Topics:
  /tts/speak        std_msgs/String   Text to speak (format: [engine|voice:]text)
  /tts/control      std_msgs/String   stop | preempt | status
  /tts/volume       std_msgs/Float32  0.0–1.0
  /tts/voice        std_msgs/String   Change voice dynamically
  /tts/status       std_msgs/String   Status info
  /tts/is_speaking  std_msgs/Bool     True while audio plays
"""

import logging
import os
import queue
import subprocess
import threading
import time
import wave
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool, Float32
from aimee_msgs.msg import AudioChunk

from aimee_tts.tts_engines import TTSEngineManager

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class TTSNode(Node):
    """Standard ROS2 TTS node with multi-engine and multi-voice support."""

    def __init__(self):
        super().__init__("tts")

        # Declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("default_engine", "lemonfox"),
                ("fallback_engine", "gtts"),
                ("auto_fallback", True),
                ("default_voice", "sarah"),
                ("kokoro_lang", "en-us"),
                ("kokoro_speed", 1.0),
                ("gtts_lang", "en"),
                ("gtts_tld", "com"),
                ("gtts_slow", False),
                ("lemonfox_api_key", ""),
                ("lemonfox_api_base_url", "https://api.lemonfox.ai/v1"),
                ("volume", 1.0),
                ("speed", 1.0),
                ("use_pygame", True),
                ("audio_device", "default"),
                ("audio_buffer_ms", 300),
                ("audio_flush_ms", 200),
                ("pygame_buffer", 2048),
                ("debug", False),
            ],
        )

        # Read parameters
        default_engine = self.get_parameter("default_engine").value
        fallback_engine = self.get_parameter("fallback_engine").value
        auto_fallback = self.get_parameter("auto_fallback").value
        default_voice = self.get_parameter("default_voice").value
        kokoro_lang = self.get_parameter("kokoro_lang").value
        kokoro_speed = self.get_parameter("kokoro_speed").value
        gtts_lang = self.get_parameter("gtts_lang").value
        gtts_tld = self.get_parameter("gtts_tld").value
        gtts_slow = self.get_parameter("gtts_slow").value
        lemonfox_api_key = self.get_parameter("lemonfox_api_key").value
        lemonfox_api_base_url = self.get_parameter("lemonfox_api_base_url").value
        volume = self.get_parameter("volume").value
        speed = self.get_parameter("speed").value
        use_pygame = self.get_parameter("use_pygame").value
        audio_device = self.get_parameter("audio_device").value
        audio_buffer_ms = self.get_parameter("audio_buffer_ms").value
        audio_flush_ms = self.get_parameter("audio_flush_ms").value
        pygame_buffer = self.get_parameter("pygame_buffer").value
        debug = self.get_parameter("debug").value

        if debug:
            self.get_logger().set_level(logging.DEBUG)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self._status_pub = self.create_publisher(String, "/tts/status", reliable_qos)
        self._speaking_pub = self.create_publisher(Bool, "/tts/is_speaking", reliable_qos)

        # Subscribers
        self.create_subscription(String, "/tts/speak", self._on_speak, 10)
        self.create_subscription(AudioChunk, "/tts/play_audio", self._on_play_audio, 10)
        self.create_subscription(String, "/tts/control", self._on_control, 10)
        self.create_subscription(Float32, "/tts/volume", self._on_volume, 10)
        self.create_subscription(String, "/tts/voice", self._on_voice, 10)

        # Engine manager
        self._manager = TTSEngineManager(
            default_engine=default_engine,
            fallback_engine=fallback_engine,
            auto_fallback=auto_fallback,
            default_voice=default_voice,
            kokoro_lang=kokoro_lang,
            kokoro_speed=kokoro_speed,
            gtts_lang=gtts_lang,
            gtts_tld=gtts_tld,
            gtts_slow=gtts_slow,
            lemonfox_api_key=lemonfox_api_key,
            lemonfox_api_base_url=lemonfox_api_base_url,
        )

        # Runtime state
        self._volume = max(0.0, min(1.0, volume))
        self._speed = speed
        self._current_voice = default_voice
        self._use_pygame = use_pygame
        self._audio_device = audio_device
        self._audio_buffer_ms = max(50, int(audio_buffer_ms))
        self._audio_flush_ms = max(50, int(audio_flush_ms))
        self._pygame_buffer = max(512, int(pygame_buffer))
        self._pygame_initialized = False
        self._is_speaking = False
        self._preempt_event = threading.Event()
        self._queue: queue.Queue = queue.Queue()
        self._speak_count = 0
        self._total_duration = 0.0
        self._consecutive_failures = 0
        self._last_health_ok = True
        self._shutdown = False

        # Streaming audio chunk accumulation
        self._audio_buffer = bytearray()
        self._audio_buffer_lock = threading.Lock()
        self._audio_buffer_timer: Optional[threading.Timer] = None
        self._audio_chunk_sample_rate = 24000
        self._audio_chunk_channels = 1

        # Gapless streaming playback state
        self._stream_channel = None
        self._stream_sounds: queue.Queue = queue.Queue()
        self._stream_sounds_lock = threading.Lock()

        # Configure ALSA device for pygame/SDL before importing pygame
        if self._audio_device and self._audio_device != "default":
            os.environ["SDL_AUDIODRIVER"] = "alsa"
            os.environ["AUDIODEV"] = self._audio_device
            self.get_logger().info(f"Pygame ALSA device set to {self._audio_device}")

        # Import pygame after environment is configured
        import pygame
        self._pygame = pygame

        # Init pygame
        if self._use_pygame:
            self._init_pygame()

        # Start worker thread
        self._worker_thread = threading.Thread(target=self._queue_worker, daemon=True)
        self._worker_thread.start()

        # Start streaming monitor thread for gapless channel playback
        self._stream_monitor_thread = threading.Thread(
            target=self._stream_monitor, daemon=True
        )
        self._stream_monitor_thread.start()

        # Status timer
        self._status_timer = self.create_timer(1.0, self._publish_status)

        voices = self._manager.get_voices()
        self.get_logger().info(
            f"TTSNode initialized:\n"
            f"  Default engine: {default_engine}\n"
            f"  Fallback: {fallback_engine}\n"
            f"  Voice: {default_voice}\n"
            f"  Audio device: {self._audio_device}\n"
            f"  Pygame buffer: {self._pygame_buffer}\n"
            f"  Available voices: {voices}\n"
            f"  Engines: {list(self._manager._engines.keys())}"
        )

    def _init_pygame(self):
        try:
            if self._pygame.mixer.get_init() is not None:
                self._pygame.mixer.quit()
            self._pygame.mixer.init(
                frequency=24000, size=-16, channels=1, buffer=self._pygame_buffer
            )
            self._pygame_initialized = True
            # Use a dedicated mixer channel for gapless cloud-audio streaming.
            # mixer.music is kept for local TTS utterances.
            self._stream_channel = self._pygame.mixer.Channel(0)
            self.get_logger().info(
                f"Pygame mixer initialized at 24kHz (buffer={self._pygame_buffer})"
            )
        except Exception as e:
            self.get_logger().warning(f"Failed to initialize pygame: {e}")
            self._pygame_initialized = False

    def _ensure_pygame(self) -> bool:
        """Reinitialize pygame mixer if it has died."""
        if not self._use_pygame:
            return False
        try:
            if self._pygame.mixer.get_init() is None:
                self.get_logger().warning("Pygame mixer died; reinitializing...")
                self._init_pygame()
        except Exception as e:
            self.get_logger().warning(f"Pygame health check failed: {e}")
            self._pygame_initialized = False
        return self._pygame_initialized

    @staticmethod
    def _parse_speak_text(text: str) -> tuple[Optional[str], Optional[str], str]:
        """Parse speak text formats:
        - text
        - engine:text
        - engine|voice:text
        """
        engine = None
        voice = None

        if ":" in text:
            prefix, rest = text.split(":", 1)
            engine_part = prefix
            if "|" in engine_part:
                engine_part, voice = engine_part.split("|", 1)
            if engine_part.lower() in ("lemonfox", "gtts", "kokoro", "auto"):
                engine = engine_part.lower()
                text = rest
                voice = voice.strip() if voice else None

        return engine, voice, text

    def _on_speak(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        engine, voice, clean_text = self._parse_speak_text(text)
        self.get_logger().info(
            f"Speaking (engine={engine or 'default'}, voice={voice or 'default'}): {clean_text[:50]}..."
        )
        self._queue.put({"text": clean_text, "engine": engine, "voice": voice})

    def _on_play_audio(self, msg: AudioChunk):
        """Handle raw PCM audio chunks for direct playback.

        Incoming chunks are accumulated and flushed as a single WAV file once
        _audio_buffer_ms of audio has been received, or after _audio_flush_ms of
        silence.  This avoids the per-chunk file I/O and pygame load latency that
        causes stuttering on small cloud audio chunks.
        """
        threshold_reached = False
        with self._audio_buffer_lock:
            self._audio_buffer.extend(bytes(msg.data))
            self._audio_chunk_sample_rate = msg.sample_rate or 24000
            self._audio_chunk_channels = msg.channels or 1

            if self._audio_buffer_timer is not None:
                self._audio_buffer_timer.cancel()
                self._audio_buffer_timer = None

            bytes_per_second = (
                self._audio_chunk_sample_rate * self._audio_chunk_channels * 2
            )
            duration_ms = (len(self._audio_buffer) / bytes_per_second) * 1000.0

            if duration_ms >= self._audio_buffer_ms:
                threshold_reached = True
            else:
                self._audio_buffer_timer = threading.Timer(
                    self._audio_flush_ms / 1000.0, self._flush_audio_buffer
                )
                self._audio_buffer_timer.daemon = True
                self._audio_buffer_timer.start()

        if threshold_reached:
            self._flush_audio_buffer()

    def _flush_audio_buffer(self):
        """Write accumulated audio chunks to a WAV file and queue for playback."""
        import tempfile

        try:
            with self._audio_buffer_lock:
                if not self._audio_buffer:
                    return
                buffer_copy = bytes(self._audio_buffer)
                sample_rate = self._audio_chunk_sample_rate
                channels = self._audio_chunk_channels
                self._audio_buffer.clear()
                if self._audio_buffer_timer is not None:
                    self._audio_buffer_timer.cancel()
                    self._audio_buffer_timer = None

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                wav_path = tmp.name

            with wave.open(wav_path, "wb") as wf:
                wf.setnchannels(channels)
                wf.setsampwidth(2)  # 16-bit
                wf.setframerate(sample_rate)
                wf.writeframes(buffer_copy)

            self._queue.put({"audio_path": wav_path, "streaming": True})
            self.get_logger().debug(
                f"Flushed {len(buffer_copy)} bytes of streaming audio "
                f"({sample_rate}Hz, {channels}ch) to {wav_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to flush audio buffer: {e}")

    def _queue_worker(self):
        """Background thread that processes the speech queue."""
        while not self._shutdown:
            try:
                req = self._queue.get(timeout=0.1)
                self._process_request(req)
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Queue worker error: {e}")

    def _process_request(self, req: dict):
        # Direct audio playback if path is provided
        if "audio_path" in req:
            if os.path.exists(req["audio_path"]):
                if req.get("streaming"):
                    self._queue_streaming_audio(req["audio_path"])
                else:
                    self._play_audio(req["audio_path"])
                try:
                    os.unlink(req["audio_path"])
                except Exception:
                    pass
            return

        text = req["text"]
        engine = req.get("engine")
        voice = req.get("voice") or self._current_voice

        # Generate audio
        result = self._manager.generate(
            text, engine_name=engine, voice=voice, speed=self._speed
        )

        if not result.success:
            self._consecutive_failures += 1
            self.get_logger().error(
                f"Speak failed: {result.error_message} (consecutive={self._consecutive_failures})"
            )
            return

        self._consecutive_failures = 0
        self._speak_count += 1
        self._total_duration += result.duration_seconds
        self.get_logger().info(f"Spoke successfully using {result.engine_used}")

        # Play audio
        if result.audio_path and os.path.exists(result.audio_path):
            self._play_audio(result.audio_path)
            try:
                os.unlink(result.audio_path)
            except Exception:
                pass

    def _play_audio(self, audio_path: str):
        self._is_speaking = True
        try:
            # Estimate max wait time from WAV duration to avoid pygame get_busy() stalls
            max_wait = 30.0
            try:
                with wave.open(audio_path, 'rb') as wf:
                    frames = wf.getnframes()
                    rate = wf.getframerate()
                    if rate > 0:
                        max_wait = max(10.0, (frames / rate) + 5.0)
            except Exception:
                pass

            if self._ensure_pygame():
                self._pygame.mixer.music.load(audio_path)
                self._pygame.mixer.music.set_volume(self._volume)
                self._pygame.mixer.music.play()
                start_time = time.time()
                while self._pygame.mixer.music.get_busy():
                    if self._preempt_event.is_set():
                        self._pygame.mixer.music.stop()
                        self._preempt_event.clear()
                        self.get_logger().info("Speech preempted")
                        break
                    if time.time() - start_time > max_wait:
                        self.get_logger().warning(
                            f"Pygame playback stalled (exceeded {max_wait:.1f}s); forcing stop"
                        )
                        self._pygame.mixer.music.stop()
                        # Reinit mixer to recover from bad state
                        self._init_pygame()
                        break
                    time.sleep(0.05)
                # Unload music to free SDL resources
                try:
                    if hasattr(self._pygame.mixer.music, 'unload'):
                        self._pygame.mixer.music.unload()
                except Exception:
                    pass
            else:
                self._play_aplay(audio_path)
        except Exception as e:
            self.get_logger().error(f"Playback failed: {e}")
            # Fallback to aplay
            self._play_aplay(audio_path)
        finally:
            self._is_speaking = False

    def _queue_streaming_audio(self, audio_path: str):
        """Queue a streaming audio WAV for gapless channel playback."""
        try:
            if not self._ensure_pygame() or not self._stream_channel:
                self.get_logger().warning(
                    "Pygame streaming channel not available; falling back to mixer.music"
                )
                self._play_audio(audio_path)
                return

            sound = self._pygame.mixer.Sound(audio_path)
            sound.set_volume(self._volume)
            with self._stream_sounds_lock:
                self._stream_sounds.put(sound)
            self._is_speaking = True
            self._drain_stream_queue()
        except Exception as e:
            self.get_logger().error(f"Failed to queue streaming audio: {e}")

    def _drain_stream_queue(self):
        """Start or queue the next streaming Sound on the dedicated channel.

        pygame's Channel.queue() plays the queued Sound immediately after the
        current one finishes, which avoids the gaps/clicks caused by reloading
        mixer.music for each chunk.
        """
        if not self._stream_channel or not self._pygame.mixer.get_init():
            return

        try:
            with self._stream_sounds_lock:
                # If nothing is playing, start the next sound.
                if not self._stream_channel.get_busy():
                    try:
                        sound = self._stream_sounds.get_nowait()
                        self._stream_channel.play(sound)
                    except queue.Empty:
                        return

                # If no sound is already queued, queue the next one.
                if not self._stream_channel.get_queue():
                    try:
                        sound = self._stream_sounds.get_nowait()
                        self._stream_channel.queue(sound)
                    except queue.Empty:
                        return
        except Exception as e:
            self.get_logger().debug(f"Stream drain error: {e}")

    def _stream_monitor(self):
        """Background thread that keeps the streaming channel fed and updates
        the speaking flag.
        """
        while not self._shutdown:
            try:
                if self._stream_channel and self._pygame.mixer.get_init():
                    self._drain_stream_queue()
                    if self._stream_channel.get_busy() or not self._stream_sounds.empty():
                        self._is_speaking = True
                    else:
                        self._is_speaking = False
                else:
                    # If pygame died, clear any pending streaming sounds.
                    with self._stream_sounds_lock:
                        while not self._stream_sounds.empty():
                            try:
                                self._stream_sounds.get_nowait()
                            except queue.Empty:
                                break
            except Exception as e:
                self.get_logger().debug(f"Stream monitor error: {e}")
            time.sleep(0.02)

    def _stop_streaming_channel(self):
        """Stop the streaming channel and discard queued sounds."""
        if self._stream_channel and self._pygame.mixer.get_init():
            try:
                self._stream_channel.stop()
            except Exception:
                pass
        with self._stream_sounds_lock:
            while not self._stream_sounds.empty():
                try:
                    self._stream_sounds.get_nowait()
                except queue.Empty:
                    break

    def _play_aplay(self, audio_path: str):
        try:
            cmd = ["aplay"]
            if self._audio_device:
                cmd.extend(["-D", self._audio_device])
            cmd.append(audio_path)
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            while proc.poll() is None:
                if self._preempt_event.is_set():
                    proc.terminate()
                    self._preempt_event.clear()
                    self.get_logger().info("Speech preempted")
                    break
                time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f"aplay failed: {e}")

    def _on_control(self, msg: String):
        command = msg.data.lower().strip()
        self.get_logger().info(f"Received control command: {command}")

        if command in ("stop", "preempt"):
            self._preempt_event.set()
            self._clear_audio_buffer()
            self._stop_streaming_channel()
            if self._pygame_initialized:
                try:
                    self._pygame.mixer.music.stop()
                except Exception:
                    pass
        elif command == "status":
            self._publish_status()
        else:
            self.get_logger().warning(f"Unknown command: {command}")

    def _clear_audio_buffer(self):
        """Discard any accumulated streaming audio chunks."""
        with self._audio_buffer_lock:
            self._audio_buffer.clear()
            if self._audio_buffer_timer is not None:
                self._audio_buffer_timer.cancel()
                self._audio_buffer_timer = None

    def _on_volume(self, msg: Float32):
        self._volume = max(0.0, min(1.0, msg.data))
        self.get_logger().info(f"Volume set to {self._volume}")

    def _on_voice(self, msg: String):
        voice = msg.data.strip()
        if voice:
            self._current_voice = voice
            self.get_logger().info(f"Voice set to {voice}")

    def _publish_status(self):
        if not self.context or not self.context.ok():
            return
        try:
            health = self._manager.health_status()
        except Exception as e:
            self.get_logger().warning(f"Health status failed: {e}")
            health = {}
        issues = []
        healthy = True

        if not self._manager._engines:
            issues.append("No TTS engines available")
            healthy = False

        try:
            if self._use_pygame and self._pygame_initialized and self._pygame.mixer.get_init() is None:
                issues.append("Pygame mixer died")
                healthy = False
        except Exception:
            issues.append("Pygame mixer check failed")
            healthy = False

        # Publish speaking state
        speaking_msg = Bool()
        speaking_msg.data = self._is_speaking
        self._speaking_pub.publish(speaking_msg)

        status_str = (
            f"engine={self._manager.default_engine}, "
            f"voice={self._current_voice}, "
            f"online={health['online']}, "
            f"speaking={self._is_speaking}, "
            f"count={self._speak_count}"
        )
        msg = String()
        msg.data = status_str
        self._status_pub.publish(msg)

        if not healthy:
            for issue in issues:
                self.get_logger().error(f"TTS health issue: {issue}")
            self._last_health_ok = False
        elif self._consecutive_failures >= 3:
            self.get_logger().error(
                f"TTS health issue: {self._consecutive_failures} consecutive speak failures"
            )
            self._last_health_ok = False
        else:
            if not self._last_health_ok:
                self.get_logger().info("TTS health recovered")
            self._last_health_ok = True

    def destroy_node(self):
        self.get_logger().info("Shutting down TTSNode...")
        self._shutdown = True
        self._status_timer.cancel()
        self._preempt_event.set()

        self._clear_audio_buffer()
        self._stop_streaming_channel()
        if self._pygame_initialized:
            try:
                self._pygame.mixer.music.stop()
                self._pygame.mixer.quit()
            except Exception:
                pass

        # Allow worker and monitor threads to exit
        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)
        if getattr(self, '_stream_monitor_thread', None) and self._stream_monitor_thread.is_alive():
            self._stream_monitor_thread.join(timeout=0.5)

        super().destroy_node()
        self.get_logger().info("Shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TTSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
