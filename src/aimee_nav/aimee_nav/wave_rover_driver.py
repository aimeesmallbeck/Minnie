#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL
#
# SPDX-License-Identifier: MPL-2.0

"""
Direct Wave Rover serial/HTTP driver.

Controls the Waveshare Wave Rover via JSON serial protocol or HTTP,
reads continuous feedback for IMU/odometry, and computes dead-reckoning.

No dependency on aimee_ugv02_controller — this driver runs entirely
in-process inside AimeeNav.

JSON Protocol:
    Move (wheel speed):  {"T":1,"L":0.5,"R":0.5}   (L/R in [-1.0, 1.0])
    Velocity:            {"T":13,"X":0.25,"Z":0.3} (m/s, rad/s)
    Continuous feedback: {"T":131,"cmd":1}          (enable T=1001 auto-feedback)
    LED:                 {"T":3,"R":255,"G":0,"B":0}

Continuous Feedback (T=1001):
    {"T":1001,"L":0,"R":0,"r":roll,"p":pitch,"y":yaw,"temp":C,"v":volts}
"""

import json
import math
import threading
import time
import urllib.request
import urllib.parse
from typing import Optional, Dict, Any, Tuple

try:
    import serial
except ImportError:
    serial = None  # type: ignore


class WaveRoverDriver:
    """
    Direct driver for Waveshare Wave Rover.

    Supports both serial and HTTP control. Computes dead-reckoning odometry
    from commanded velocities (since Wave Rover has no wheel encoders).
    """

    CMD_SPEED_CTRL = 1
    CMD_VELOCITY = 13
    CMD_CONTINUOUS_FEEDBACK = 131
    CMD_LED = 3

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 115200,
        timeout: float = 1.0,
        http_ip: str = '',
        wheel_separation: float = 0.172,
        wheel_radius: float = 0.04,
        max_speed: float = 0.5,
        max_angular: float = 1.5,
        angular_scale: float = 1.0,
        control_mode: str = 'wheel_speed',
        min_http_interval: float = 0.2,
        cmd_timeout: float = 0.5,
        accel_limit_linear: float = 0.0,
        accel_limit_angular: float = 0.0,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._http_ip = http_ip
        self._wheel_sep = wheel_separation
        self._wheel_radius = wheel_radius
        self._max_speed = max_speed
        self._max_angular = max_angular
        self._angular_scale = angular_scale
        self._control_mode = control_mode
        self._min_http_interval = min_http_interval
        self._cmd_timeout = cmd_timeout
        self._accel_limit_linear = accel_limit_linear
        self._accel_limit_angular = accel_limit_angular

        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None

        # HTTP rate limiting
        self._http_lock = threading.Lock()
        self._last_http_send_time = 0.0
        self._http_timeout = 1.0

        # Command state
        self._last_cmd_time = time.time()
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_cmd_L = 0.0
        self._last_cmd_R = 0.0
        self._cmd_active = False

        # Odometry state (dead reckoning)
        self._odom_lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._vx = 0.0
        self._vth = 0.0
        self._last_odom_time = time.time()

        # IMU state
        self._imu_lock = threading.Lock()
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._battery_voltage = 0.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open serial connection and start feedback reader."""
        if self._http_ip:
            # HTTP mode: no serial needed, but we still need to enable feedback
            self._running = True
            return True

        if serial is None:
            raise RuntimeError("pyserial is not installed")

        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout,
            )
            self._serial.setRTS(False)
            self._serial.setDTR(False)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open rover serial port {self._port}: {e}")

        self._running = True
        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        # Enable continuous feedback
        self._enable_continuous_feedback(True)
        return True

    def disconnect(self) -> None:
        """Stop robot and close connection."""
        self.stop()
        self._running = False
        if self._read_thread is not None:
            self._read_thread.join(timeout=1.0)
            self._read_thread = None
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
            self._serial = None

    def is_connected(self) -> bool:
        if self._http_ip:
            return True
        return self._serial is not None and self._serial.is_open

    # ------------------------------------------------------------------
    # High-level motion commands
    # ------------------------------------------------------------------

    def send_velocity(self, linear_x: float, angular_z: float) -> None:
        """Send a velocity command to the rover."""
        if self._max_speed <= 0:
            # Zero-speed mode — just send stop and update dead-reckoning
            self._last_cmd_time = time.time()
            self._last_linear = 0.0
            self._last_angular = 0.0
            self._cmd_active = False
            self._update_odometry(0.0, 0.0)
            if self._control_mode == 'wheel_speed':
                self._send_command({"T": self.CMD_SPEED_CTRL, "L": 0.0, "R": 0.0})
            else:
                self._send_command({"T": self.CMD_VELOCITY, "X": 0.0, "Z": 0.0})
            return

        linear_x = max(-self._max_speed, min(self._max_speed, linear_x))
        # Do NOT clamp angular here — let wheel-speed normalization handle it,
        # matching the behaviour of aimee_ugv02_controller.
        # NOTE: angular_scale is applied inside the wheel-speed formula below;
        # do NOT pre-scale here to avoid double-scaling.

        # ─── Velocity smoothing (ramp-rate limiting) ───
        dt = time.time() - self._last_cmd_time
        if dt > 0.0 and self._cmd_active:
            # Cap dt to avoid huge jumps after long pauses
            dt = min(dt, 0.5)
            if self._accel_limit_linear > 0.0:
                delta = linear_x - self._last_linear
                max_delta = self._accel_limit_linear * dt
                if abs(delta) > max_delta:
                    linear_x = self._last_linear + math.copysign(max_delta, delta)
            if self._accel_limit_angular > 0.0:
                delta = angular_z - self._last_angular
                max_delta = self._accel_limit_angular * dt
                if abs(delta) > max_delta:
                    angular_z = self._last_angular + math.copysign(max_delta, delta)

        self._last_cmd_time = time.time()
        self._last_linear = linear_x
        self._last_angular = angular_z
        self._cmd_active = True

        # Log smoothed velocity for diagnostics
        if linear_x != 0.0 or angular_z != 0.0:
            print(f"[WaveRover] smoothed linear={linear_x:.3f} angular={angular_z:.3f}", flush=True)

        # Update dead-reckoning immediately (since no encoders)
        self._update_odometry(linear_x, angular_z)

        if self._control_mode == 'wheel_speed':
            # ESP32 firmware accepts T=1 with L/R in [-1.0, 1.0].
            # We use a scaled differential formula that accounts for the
            # Wave Rover motor dead zone (~0.18 power minimum).
            fwd = linear_x / self._max_speed
            diff = angular_z / self._max_speed * self._angular_scale

            L = fwd - diff
            R = fwd + diff

            # Motor dead-zone compensation: if either wheel command is
            # non-zero but below the motor's minimum reliable power,
            # scale the entire (L, R) vector up so BOTH wheels reach
            # the threshold. This preserves turn geometry while ensuring
            # the robot actually moves.
            MIN_POWER = 0.18
            if abs(L) > 1e-4 or abs(R) > 1e-4:
                scale_L = MIN_POWER / abs(L) if 1e-4 < abs(L) < MIN_POWER else 1.0
                scale_R = MIN_POWER / abs(R) if 1e-4 < abs(R) < MIN_POWER else 1.0
                scale = max(scale_L, scale_R)
                L *= scale
                R *= scale

            # Clamp to firmware's full range [-1.0, 1.0]
            L = max(-1.0, min(1.0, L))
            R = max(-1.0, min(1.0, R))

            self._last_cmd_L = L
            self._last_cmd_R = R
            cmd = {"T": self.CMD_SPEED_CTRL, "L": round(L, 4), "R": round(R, 4)}
        else:
            cmd = {"T": self.CMD_VELOCITY, "X": round(linear_x, 3), "Z": round(angular_z, 3)}
            self._last_cmd_L = 0.0
            self._last_cmd_R = 0.0

        self._send_command(cmd)

    def stop(self) -> None:
        """Send zero velocity command."""
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_cmd_L = 0.0
        self._last_cmd_R = 0.0
        self._cmd_active = False
        self.send_velocity(0.0, 0.0)

    def set_led(self, r: int, g: int, b: int) -> None:
        """Set rover LED color."""
        self._send_command({"T": self.CMD_LED, "R": r, "G": g, "B": b})

    # ------------------------------------------------------------------
    # Odometry & IMU accessors
    # ------------------------------------------------------------------

    def get_odometry(self) -> Tuple[float, float, float, float, float]:
        """Return (x, y, theta, vx, vth) — thread-safe."""
        with self._odom_lock:
            return (self._x, self._y, self._theta, self._vx, self._vth)

    def get_imu(self) -> Tuple[float, float, float]:
        """Return (roll, pitch, yaw) in radians — thread-safe."""
        with self._imu_lock:
            return (math.radians(self._roll), math.radians(self._pitch), math.radians(self._yaw))

    def get_battery_voltage(self) -> float:
        with self._imu_lock:
            return self._battery_voltage

    def reset_odometry(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        with self._odom_lock:
            self._x = x
            self._y = y
            self._theta = theta
            self._last_odom_time = time.time()

    def check_watchdog(self) -> bool:
        """Return True if command timeout occurred and robot was stopped."""
        if not self._cmd_active:
            return False
        if time.time() - self._last_cmd_time > self._cmd_timeout:
            self.stop()
            return True
        return False

    # ------------------------------------------------------------------
    # Internal: serial communication
    # ------------------------------------------------------------------

    def _send_command(self, cmd: Dict[str, Any]) -> None:
        if self._http_ip:
            self._send_http(cmd)
        else:
            self._send_serial(cmd)

    def _send_serial(self, cmd: Dict[str, Any]) -> None:
        if self._serial is None or not self._serial.is_open:
            return
        try:
            with self._serial_lock:
                payload = json.dumps(cmd, separators=(',', ':')) + '\n'
                self._serial.write(payload.encode('utf-8'))
                self._serial.flush()
        except serial.SerialException:
            pass

    def _send_http(self, cmd: Dict[str, Any]) -> None:
        with self._http_lock:
            now = time.time()
            dt = now - self._last_http_send_time
            # Always allow stop commands through — never drop a zero-velocity emergency stop
            is_stop = False
            if self._control_mode == 'wheel_speed':
                is_stop = (cmd.get('L', 0.0) == 0.0 and cmd.get('R', 0.0) == 0.0)
            else:
                is_stop = (cmd.get('X', 0.0) == 0.0 and cmd.get('Z', 0.0) == 0.0)
            if dt < self._min_http_interval and not is_stop:
                return
            # Mark send time immediately so the nav loop isn't gated by HTTP latency
            self._last_http_send_time = now
        # Offload blocking I/O to a background thread
        threading.Thread(target=self._send_http_blocking, args=(cmd,), daemon=True).start()

    def _send_http_blocking(self, cmd: Dict[str, Any]) -> None:
        try:
            json_str = json.dumps(cmd, separators=(',', ':'))
            encoded = urllib.parse.quote(json_str, safe='')
            url = f"http://{self._http_ip}/js?json={encoded}"
            req = urllib.request.Request(url, headers={'Connection': 'close'})
            with urllib.request.urlopen(req, timeout=self._http_timeout) as resp:
                resp.read()
        except Exception:
            pass

    def _enable_continuous_feedback(self, enable: bool) -> None:
        self._send_command({"T": self.CMD_CONTINUOUS_FEEDBACK, "cmd": 1 if enable else 0})

    # ------------------------------------------------------------------
    # Internal: serial read loop
    # ------------------------------------------------------------------

    def _serial_read_loop(self) -> None:
        """Background thread: read JSON feedback from ESP32."""
        while self._running and self._serial is not None and self._serial.is_open:
            try:
                with self._serial_lock:
                    if self._serial.in_waiting > 0:
                        line = self._serial.readline().decode('utf-8').strip()
                        if line:
                            self._process_feedback(line)
            except Exception:
                pass
            time.sleep(0.02)

    def _process_feedback(self, line: str) -> None:
        """Parse JSON feedback from rover."""
        try:
            msg = json.loads(line)
            cmd_type = msg.get('T')
            if cmd_type == 1001:
                self._process_continuous_feedback(msg)
        except (json.JSONDecodeError, ValueError):
            pass

    def _process_continuous_feedback(self, msg: Dict[str, Any]) -> None:
        """Process T=1001 continuous feedback packet."""
        try:
            # Update odometry from commanded velocities (no encoders on Wave Rover)
            left_speed = msg.get('L', 0)
            right_speed = msg.get('R', 0)

            if self._control_mode == 'wheel_speed':
                # L/R are now in [-1.0, 1.0] representing full motor range
                v_left = self._last_cmd_L * self._max_speed
                v_right = self._last_cmd_R * self._max_speed
            else:
                v_left = left_speed * self._wheel_radius
                v_right = right_speed * self._wheel_radius

            vx = (v_right + v_left) / 2.0
            vth = (v_right - v_left) / self._wheel_sep

            with self._odom_lock:
                self._vx = vx
                self._vth = vth

            # IMU data
            with self._imu_lock:
                self._roll = msg.get('r', 0.0)
                self._pitch = msg.get('p', 0.0)
                self._yaw = msg.get('y', 0.0)
                self._battery_voltage = msg.get('v', 0.0)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Internal: dead reckoning
    # ------------------------------------------------------------------

    def _update_odometry(self, linear_x: float, angular_z: float) -> None:
        """Store commanded velocity but do NOT integrate pose.

        The Wave Rover has no wheel encoders. Integrating commanded
        velocities creates fake odometry that corrupts EKF state when
        fused with scan matching + IMU. Pose is now tracked exclusively
        by the EKF (scan matches + IMU gyro).
        """
        with self._odom_lock:
            self._last_odom_time = time.time()
            self._vx = linear_x
            self._vth = angular_z
            # NOTE: Disabled pose integration — robot has no wheel encoders.
            # self._x += linear_x * math.cos(self._theta) * dt
            # self._y += linear_x * math.sin(self._theta) * dt
            # self._theta += angular_z * dt
