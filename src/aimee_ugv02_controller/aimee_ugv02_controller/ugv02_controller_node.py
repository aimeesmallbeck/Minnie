#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL
#
# SPDX-License-Identifier: MPL-2.0

"""
UGV02 Platform Controller Node

Controls the Waveshare UGV02 rover via JSON serial protocol.
Subscribes to /cmd_vel, publishes odometry, handles ESP32 communication.

JSON Protocol (115200 baud, 8N1):
- Move:        {"T":1,"L":0.5,"R":0.5}  (L/R wheel speeds -0.5 to 0.5)
- Velocity:    {"T":13,"X":0.25,"Z":0.3} (linear X, angular Z)
- Odometry:    {"T":130} → chassis feedback
- IMU:         {"T":126} → IMU data
- LED:         {"T":3,"R":255,"G":0,"B":0}
- Continuous:  {"T":131,"cmd":1} (enable auto-feedback for ROS)

Usage:
    ros2 run aimee_ugv02_controller ugv02_controller_node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Float32, String, Bool
import tf2_ros
import serial
import json
import threading
import time
import math
import urllib.request
import urllib.parse
from typing import Optional, Dict, Any


class UGV02ControllerNode(Node):
    """
    ROS2 node for UGV02 rover control via JSON serial protocol.
    
    Publishes:
        - /odom (Odometry): Wheel odometry from ESP32
        - /imu (ImU): IMU data from ESP32
        - /battery (BatteryState): Battery voltage
        - /tf (TF): odom → base_link transform
    
    Subscribes:
        - /cmd_vel (Twist): Velocity commands
    """

    # Command types
    CMD_SPEED_CTRL = 1       # Direct wheel speed control
    CMD_VELOCITY = 13        # Linear/angular velocity
    CMD_ODOMETRY = 130       # Get chassis feedback
    CMD_CONTINUOUS_FEEDBACK = 131  # Enable/disable continuous feedback
    CMD_IMU = 126            # Get IMU data
    CMD_LED = 3              # LED control
    CMD_OLED = 3             # OLED display (lineNum, Text)
    CMD_ECHO = 143           # Echo switch

    def __init__(self):
        super().__init__('ugv02_controller')

        # Declare parameters
        self.declare_parameters(namespace='', parameters=[
            ('serial_port', '/dev/ttyACM0'),
            ('baud_rate', 115200),
            ('timeout', 1.0),
            ('base_frame', 'base_link'),
            ('odom_frame', 'odom'),
            ('wheel_separation', 0.23),    # meters (distance between wheels)
            ('wheel_radius', 0.04),        # meters
            ('max_speed', 0.5),            # m/s
            ('cmd_timeout', 0.5),          # seconds before stopping
            ('heartbeat_interval', 0.5),   # seconds between heartbeat commands
            ('continuous_feedback', True), # Enable ESP32 continuous feedback
            ('publish_tf', True),
            ('linear_scale', 1.0),         # Scale factor for linear velocity
            ('angular_scale', 1.0),        # Scale factor for angular velocity
            ('control_mode', 'velocity'),  # 'velocity' (T=13) or 'wheel_speed' (T=1)
            ('http_ip', ''),               # ESP32 IP for HTTP commands (e.g., 192.168.1.56)
            ('use_serial', True),          # Set False for pure Wi-Fi HTTP mode
        ])

        # Get parameters
        self._serial_port = self.get_parameter('serial_port').value
        self._baud_rate = self.get_parameter('baud_rate').value
        self._timeout = self.get_parameter('timeout').value
        self._base_frame = self.get_parameter('base_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._wheel_sep = self.get_parameter('wheel_separation').value
        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._max_speed = self.get_parameter('max_speed').value
        self._cmd_timeout = self.get_parameter('cmd_timeout').value
        self._heartbeat_interval = self.get_parameter('heartbeat_interval').value
        self._continuous_feedback = self.get_parameter('continuous_feedback').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._linear_scale = self.get_parameter('linear_scale').value
        self._angular_scale = self.get_parameter('angular_scale').value
        self._control_mode = self.get_parameter('control_mode').value
        self._http_ip = self.get_parameter('http_ip').value
        self._use_serial = self.get_parameter('use_serial').value
        self._http_mode = bool(self._http_ip)

        # Setup QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, '/odom', odom_qos)
        self._imu_pub = self.create_publisher(Imu, '/imu', odom_qos)
        self._battery_pub = self.create_publisher(BatteryState, '/battery', reliable_qos)
        self._status_pub = self.create_publisher(String, '/ugv02/status', reliable_qos)

        # TF broadcaster
        if self._publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10
        )

        # Serial connection
        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()

        # State
        self._connected = False
        self._last_cmd_time = time.time()
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._cmd_vel_active = False

        # HTTP rate limiting (lessons from RoArm-M3 driver)
        self._http_lock = threading.Lock()
        self._last_http_send_time = 0.0
        self._min_http_interval = 0.2  # max 5 Hz — ESP32 web server can't sustain more
        self._http_timeout = 1.0

        # Odometry state
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._vx = 0.0
        self._vth = 0.0
        self._last_odom_time = time.time()
        
        # Publish decimation (process all, publish every Nth)
        self._feedback_count = 0
        self._publish_decimation = 1  # Publish at ~10 Hz from 50 Hz input

        # Serial read thread
        self._read_thread: Optional[threading.Thread] = None
        self._running = False

        # Timers
        self._heartbeat_timer = None
        self._watchdog_timer = None
        self._odom_timer = None

        # Connect to serial if requested
        if self._use_serial:
            self._connect_serial()
        else:
            self.get_logger().info("Serial disabled; running in HTTP-only mode")

        if self._connected and self._continuous_feedback:
            self._enable_continuous_feedback(True)

        # Start timers for HTTP mode and/or serial mode
        if self._http_mode or self._connected:
            self._heartbeat_timer = self.create_timer(
                self._heartbeat_interval, self._heartbeat_callback
            )
            self._watchdog_timer = self.create_timer(
                0.1, self._watchdog_callback
            )

        # Odometry publisher: integrate commanded (or feedback) velocities at 10 Hz
        self._odom_timer = self.create_timer(0.1, self._odom_callback)

        mode_str = "HTTP" if self._http_mode else "serial"
        self.get_logger().info(
            f"UGV02 Controller initialized:\n"
            f"  Mode: {mode_str}\n"
            f"  Serial: {self._serial_port if self._use_serial else 'disabled'} @ {self._baud_rate} baud\n"
            f"  Base frame: {self._base_frame}\n"
            f"  Odom frame: {self._odom_frame}\n"
            f"  Max speed: {self._max_speed} m/s\n"
            f"  Connected: {self._connected}"
        )

    def _connect_serial(self) -> bool:
        """Connect to the ESP32 via serial."""
        try:
            self._serial = serial.Serial(
                port=self._serial_port,
                baudrate=self._baud_rate,
                timeout=self._timeout,
                dsrdtr=None
            )
            self._serial.setRTS(False)
            self._serial.setDTR(False)
            
            # Clear buffers
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            
            self._connected = True
            
            # Start read thread
            self._running = True
            self._read_thread = threading.Thread(target=self._serial_read_loop)
            self._read_thread.daemon = True
            self._read_thread.start()
            
            self.get_logger().info(f"Connected to {self._serial_port}")
            return True
            
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self._serial_port}: {e}")
            self._connected = False
            return False

    def _serial_read_loop(self):
        """Background thread to read serial data."""
        while self._running and self._serial and self._serial.is_open:
            try:
                with self._serial_lock:
                    if self._serial.in_waiting > 0:
                        line = self._serial.readline().decode('utf-8').strip()
                        if line:
                            self._process_serial_data(line)
            except Exception as e:
                self.get_logger().debug(f"Serial read error: {e}")
            time.sleep(0.02)  # Small delay to prevent CPU spinning

    def _process_serial_data(self, data: str):
        """Process incoming JSON data from ESP32."""
        try:
            msg = json.loads(data)
            cmd_type = msg.get('T')
            
            if cmd_type == 130:  # Base feedback
                self._process_odometry(msg)
            elif cmd_type == 126:  # IMU data
                self._process_imu(msg)
            elif cmd_type == 2:  # Legacy odometry
                self._process_legacy_odometry(msg)
            elif cmd_type == 1001:  # Wave Rover continuous feedback
                self._process_continuous_feedback(msg)
            else:
                # Log other messages for debugging
                self.get_logger().debug(f"Received: {data}")
                
        except json.JSONDecodeError:
            self.get_logger().debug(f"Non-JSON data: {data}")
        except Exception as e:
            self.get_logger().debug(f"Error processing data: {e}")

    def _process_continuous_feedback(self, msg: Dict[str, Any]):
        """Process Wave Rover T=1001 continuous feedback packet.

        Format: {"T":1001,"L":0,"R":0,"r":roll,"p":pitch,"y":yaw,"temp":C,"v":volts}
        """
        try:
            # Extract wheel speeds for odometry
            if self._control_mode == 'wheel_speed':
                # Wave Rover has no encoders, so T=1001 L/R are always 0.
                # Use the last commanded L/R values (in [-0.5, 0.5]) and convert to m/s.
                left_speed = self._last_cmd_L * 2.0  # [-0.5, 0.5] -> [-1, 1]
                right_speed = self._last_cmd_R * 2.0
                v_left = left_speed * self._max_speed
                v_right = right_speed * self._max_speed
            else:
                left_speed = msg.get('L', 0)
                right_speed = msg.get('R', 0)
                v_left = left_speed * self._wheel_radius
                v_right = right_speed * self._wheel_radius

            self._vx = (v_right + v_left) / 2.0
            self._vth = (v_right - v_left) / self._wheel_sep

            # Odometry integration is handled by _odom_callback so it works
            # identically with or without serial feedback.

            # Publish IMU if r/p/y present
            if 'r' in msg or 'p' in msg or 'y' in msg:
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'

                # r/p/y are in degrees from Wave Rover
                roll_rad = math.radians(msg.get('r', 0))
                pitch_rad = math.radians(msg.get('p', 0))
                yaw_rad = math.radians(msg.get('y', 0))
                imu_msg.orientation = self._euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

                self._imu_pub.publish(imu_msg)

            # Publish battery voltage if present
            if 'v' in msg:
                battery_msg = BatteryState()
                battery_msg.header.stamp = self.get_clock().now().to_msg()
                battery_msg.voltage = float(msg['v'])
                battery_msg.present = True
                self._battery_pub.publish(battery_msg)

        except Exception as e:
            self.get_logger().debug(f"Continuous feedback processing error: {e}")

    def _process_odometry(self, msg: Dict[str, Any]):
        """Process odometry feedback from ESP32."""
        # Parse feedback - exact format depends on ESP32 firmware
        # Typical format: {"T":130, "L":left_speed, "R":right_speed, 
        #                  "X":x_pos, "Y":y_pos, "Z":heading}
        
        try:
            left_speed = msg.get('L', 0)  # Left wheel speed
            right_speed = msg.get('R', 0)  # Right wheel speed

            # Calculate robot velocities
            # v = (v_r + v_l) / 2
            # omega = (v_r - v_l) / wheel_separation
            v_left = left_speed * self._wheel_radius
            v_right = right_speed * self._wheel_radius

            self._vx = (v_right + v_left) / 2.0
            self._vth = (v_right - v_left) / self._wheel_sep

            # Odometry integration is handled by _odom_callback.

        except Exception as e:
            self.get_logger().debug(f"Odometry processing error: {e}")

    def _process_legacy_odometry(self, msg: Dict[str, Any]):
        """Process legacy odometry format (T=2)."""
        # Format: {"T":2,"L":100,"R":100,"X":10,"Y":5,"Z":90}
        try:
            # This appears to be encoder counts or PWM values
            # Convert to velocities if needed
            pass
        except Exception as e:
            self.get_logger().debug(f"Legacy odometry error: {e}")

    def _process_imu(self, msg: Dict[str, Any]):
        """Process IMU data from ESP32."""
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Parse IMU data - exact fields depend on firmware
            # Typical: heading, pitch, roll, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
            if 'heading' in msg:
                # Convert heading to quaternion (simplified - assuming 2D)
                heading_rad = math.radians(msg['heading'])
                imu_msg.orientation = self._euler_to_quaternion(0, 0, heading_rad)
            
            if 'accel_x' in msg:
                imu_msg.linear_acceleration.x = msg['accel_x']
                imu_msg.linear_acceleration.y = msg.get('accel_y', 0)
                imu_msg.linear_acceleration.z = msg.get('accel_z', 0)
            
            if 'gyro_x' in msg:
                imu_msg.angular_velocity.x = msg['gyro_x']
                imu_msg.angular_velocity.y = msg.get('gyro_y', 0)
                imu_msg.angular_velocity.z = msg.get('gyro_z', 0)
            
            self._imu_pub.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().debug(f"IMU processing error: {e}")

    def _publish_odometry(self):
        """Publish odometry message and TF."""
        now = self.get_clock().now()
        
        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        
        # Position
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._euler_to_quaternion(0, 0, self._theta)
        
        # Velocity
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.angular.z = self._vth
        
        # Covariances (set high since we don't have covariance estimates)
        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.2
        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[35] = 0.1
        
        self._odom_pub.publish(odom)
        
        # Publish TF
        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(t)

    def _odom_callback(self):
        """Integrate velocities and publish odometry at a fixed rate.

        Works with or without serial feedback: _vx/_vth are set by cmd_vel
        and updated by feedback packets when available.
        """
        now = time.time()
        dt = now - self._last_odom_time
        if dt > 0 and dt < 1.0:  # ignore large jumps (e.g., startup)
            delta_x = self._vx * math.cos(self._theta) * dt
            delta_y = self._vx * math.sin(self._theta) * dt
            delta_th = self._vth * dt

            self._x += delta_x
            self._y += delta_y
            self._theta += delta_th
            self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        self._last_odom_time = now
        self._publish_odometry()

    def _on_cmd_vel(self, msg: Twist):
        """Handle incoming velocity commands."""
        self._last_cmd_time = time.time()
        self._cmd_vel_active = True

        # Extract velocities
        linear_x = msg.linear.x * self._linear_scale
        # angular_scale is applied in _send_velocity_command; don't double-scale here
        angular_z = msg.angular.z

        # Clamp linear to max speed
        linear_x = max(-self._max_speed, min(self._max_speed, linear_x))
        # Note: angular is NOT clamped here - _send_velocity_command normalizes wheel speeds

        self._last_linear = linear_x
        self._last_angular = angular_z

        # Use commanded velocities as the odometry source when no encoder feedback is available
        self._vx = linear_x
        self._vth = angular_z

        # Send to robot
        self.get_logger().info(f"CMD_VEL received: linear={linear_x:.2f} angular={angular_z:.2f}")
        self._send_velocity_command(linear_x, angular_z)

    def _send_velocity_command(self, linear_x: float, angular_z: float):
        """Send velocity command to ESP32 via HTTP or serial."""
        # Update the velocities used for open-loop odometry integration
        self._vx = linear_x
        self._vth = angular_z

        if self._control_mode == 'wheel_speed':
            # Wave Rover ESP32 firmware expects T=1 with L/R as direct motor
            # commands in [-0.5, 0.5], matching the web teleop's movtionButton().
            # Web teleop mixing:  L = fwd - diff,  R = fwd + diff
            #
            # fwd  = linear component  in [-0.5, 0.5]
            # diff = angular component in [-0.5, 0.5]
            #
            # Pure forward at max_speed  → L=0.5, R=0.5
            # Pure turn  at max_angular  → L=-0.5, R=0.5 (or vice versa)
            fwd = linear_x / self._max_speed * 0.5
            diff = angular_z / self._max_speed * 0.5 * self._angular_scale

            # Prevent inner wheel reversal when moving forward/backward.
            # The web teleop keeps both wheels with the same sign as fwd
            # for combined motion (e.g. forward-left: L=0.3, R=0.5).
            if abs(fwd) > 0.001 and abs(diff) > abs(fwd):
                diff = math.copysign(abs(fwd), diff)

            L = fwd - diff
            R = fwd + diff

            # Clamp to firmware's expected range [-0.5, 0.5]
            L = max(-0.5, min(0.5, L))
            R = max(-0.5, min(0.5, R))

            self.get_logger().info(f"CMD: L={L:.3f} R={R:.3f}")
            cmd = {"T": self.CMD_SPEED_CTRL, "L": round(L, 4), "R": round(R, 4)}
            self._last_cmd_L = L
            self._last_cmd_R = R
        else:
            # UGV01 with encoders: use T=13 with X (linear m/s) and Z (angular rad/s)
            linear_x = max(-self._max_speed, min(self._max_speed, linear_x))
            angular_z = max(-self._max_speed, min(self._max_speed, angular_z))

            self.get_logger().info(f"CMD: X={linear_x:.2f} Z={angular_z:.2f}")
            cmd = {"T": self.CMD_VELOCITY, "X": linear_x, "Z": angular_z}
            self._last_cmd_L = 0.0
            self._last_cmd_R = 0.0
        
        if self._http_ip:
            self._send_http_command(cmd)
        else:
            self._send_json(cmd)

    def _send_velocity_command_alt(self, linear_x: float, angular_z: float):
        """Alternative: Use T=13 velocity command."""
        if not self._connected or not self._serial:
            return
        
        self.get_logger().info(f"Sending T=13: X={linear_x:.2f} Z={angular_z:.2f}")
        cmd = {"T": self.CMD_VELOCITY, "X": linear_x, "Z": angular_z}
        self._send_json(cmd)

    def _send_http_command(self, cmd: Dict[str, Any]):
        """Send JSON command to ESP32 via HTTP GET.

        Rate-limited to max 5 Hz because the ESP32 web server cannot sustain
        high-frequency requests. Uses fresh connections (Connection: close) to
        avoid crashing the ESP32's lightweight HTTP stack.
        """
        with self._http_lock:
            now = time.time()
            if now - self._last_http_send_time < self._min_http_interval:
                return  # Too soon — drop to prevent ESP32 overload

            try:
                json_str = json.dumps(cmd, separators=(',', ':'))
                encoded = urllib.parse.quote(json_str, safe='')
                url = f"http://{self._http_ip}/js?json={encoded}"
                req = urllib.request.Request(
                    url, headers={'Connection': 'close'}
                )
                with urllib.request.urlopen(req, timeout=self._http_timeout) as resp:
                    resp.read()
                self._last_http_send_time = now
                self.get_logger().info(f"HTTP sent: {json_str}")
            except Exception as e:
                self.get_logger().warn(f"HTTP command error: {e}")

    def _send_json(self, cmd: Dict[str, Any]):
        """Send JSON command to ESP32 via serial."""
        if not self._connected or not self._serial:
            return
        try:
            with self._serial_lock:
                json_str = json.dumps(cmd) + '\n'
                self.get_logger().info(f"Serial write: {json_str.strip()}")
                self._serial.write(json_str.encode('utf-8'))
                self._serial.flush()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            self._connected = False

    def _enable_continuous_feedback(self, enable: bool):
        """Enable or disable continuous feedback from ESP32."""
        cmd = {"T": self.CMD_CONTINUOUS_FEEDBACK, "cmd": 1 if enable else 0}
        self._send_json(cmd)
        self.get_logger().info(f"Continuous feedback {'enabled' if enable else 'disabled'}")

    def _heartbeat_callback(self):
        """Send periodic heartbeat to keep robot moving.

        Skips if a command was recently sent via HTTP (the ESP32 already has
        a fresh command). This avoids piling up requests on the ESP32 server.
        """
        if not self._http_mode and not self._connected:
            return

        # Skip heartbeat if we just sent a command via HTTP (let ESP32 recover)
        if time.time() - self._last_http_send_time < self._min_http_interval:
            return

        # If we have an active command, resend it to prevent auto-stop
        if self._cmd_vel_active:
            self._send_velocity_command(self._last_linear, self._last_angular)

    def _watchdog_callback(self):
        """Watchdog to stop robot if no commands received."""
        time_since_last_cmd = time.time() - self._last_cmd_time
        
        if time_since_last_cmd > self._cmd_timeout and self._cmd_vel_active:
            self.get_logger().warn("Command timeout - stopping robot")
            self._send_velocity_command(0.0, 0.0)
            self._cmd_vel_active = False
            self._last_linear = 0.0
            self._last_angular = 0.0

    def set_led(self, r: int, g: int, b: int):
        """Set LED color."""
        cmd = {"T": self.CMD_LED, "R": r, "G": g, "B": b}
        self._send_json(cmd)

    def set_oled(self, line: int, text: str):
        """Set OLED display text."""
        cmd = {"T": self.CMD_OLED, "lineNum": line, "Text": text[:16]}  # Limit to 16 chars
        self._send_json(cmd)

    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert Euler angles to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q

    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info("Shutting down UGV02 controller...")

        self._running = False

        # Cancel timers
        for timer in (self._heartbeat_timer, self._watchdog_timer, self._odom_timer):
            if timer is not None:
                timer.cancel()

        # Stop robot (HTTP or serial)
        if self._http_mode or self._connected:
            self._send_velocity_command(0.0, 0.0)
            time.sleep(0.1)

        # Close serial
        if self._serial and self._serial.is_open:
            self._serial.close()

        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UGV02ControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
