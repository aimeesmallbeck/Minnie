#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
#
# SPDX-License-Identifier: MPL-2.0

"""
Animation behavior node for the AIMEE Robot base.

Generates small, slow body motions:
  - gentle back-and-forth sway while TTS is speaking
  - small random movement after a configurable idle timeout
  - returns to the starting pose if displacement exceeds a safety radius

Publishes low-priority /cmd_vel commands and pauses immediately whenever an
external /cmd_vel command is received (so it never fights navigation or teleop).
"""

import math
import random
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class AnimationNode(Node):
    """ROS2 node that adds idle and speaking animations to the base."""

    STATE_DISABLED = "disabled"
    STATE_IDLE = "idle"
    STATE_TALKING = "talking"
    STATE_ANIMATING = "animating"
    STATE_RETURNING_HOME = "returning_home"

    def __init__(self):
        super().__init__("animation_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("enabled", True),
                ("idle_timeout_s", 15.0),
                ("max_displacement_m", 0.12),
                ("talk_sway_speed_m_s", 0.03),
                ("talk_sway_period_s", 1.5),
                ("animation_linear_speed_m_s", 0.04),
                ("animation_angular_speed_rad_s", 0.15),
                ("animation_move_duration_s", 1.5),
                ("cmd_vel_timeout_s", 0.5),
                ("odom_timeout_s", 1.0),
                ("home_marker_id", 0),
                ("return_home_linear_speed_m_s", 0.05),
                ("return_home_angular_speed_rad_s", 0.2),
                ("marker_visible_timeout_s", 3.0),
                ("random_seed", -1),
                ("talk_animation_mode", "random"),  # "random" small moves or "sway"
                ("talk_move_distance_min_m", 0.05),  # ~2 in
                ("talk_move_distance_max_m", 0.15),  # ~6 in
                ("talk_turn_angle_min_deg", 5.0),
                ("talk_turn_angle_max_deg", 15.0),
                ("talk_move_speed_m_s", 0.03),
                ("talk_turn_speed_rad_s", 0.15),
                ("talk_move_interval_s", 1.5),
            ],
        )

        self._enabled = self.get_parameter("enabled").value
        self._idle_timeout_s = max(1.0, self.get_parameter("idle_timeout_s").value)
        self._max_displacement_m = max(0.02, self.get_parameter("max_displacement_m").value)
        self._talk_sway_speed = abs(self.get_parameter("talk_sway_speed_m_s").value)
        self._talk_sway_period = max(0.5, self.get_parameter("talk_sway_period_s").value)
        self._anim_linear_speed = abs(self.get_parameter("animation_linear_speed_m_s").value)
        self._anim_angular_speed = abs(self.get_parameter("animation_angular_speed_rad_s").value)
        self._anim_duration = max(0.5, self.get_parameter("animation_move_duration_s").value)
        self._cmd_vel_timeout = max(0.1, self.get_parameter("cmd_vel_timeout_s").value)
        self._odom_timeout = max(0.5, self.get_parameter("odom_timeout_s").value)
        self._home_marker_id = self.get_parameter("home_marker_id").value
        self._return_linear_speed = abs(self.get_parameter("return_home_linear_speed_m_s").value)
        self._return_angular_speed = abs(self.get_parameter("return_home_angular_speed_rad_s").value)
        self._marker_visible_timeout = max(1.0, self.get_parameter("marker_visible_timeout_s").value)

        self._talk_animation_mode = self.get_parameter("talk_animation_mode").value
        self._talk_move_distance_min = max(0.01, self.get_parameter("talk_move_distance_min_m").value)
        self._talk_move_distance_max = max(self._talk_move_distance_min, self.get_parameter("talk_move_distance_max_m").value)
        self._talk_turn_angle_min = max(0.0, self.get_parameter("talk_turn_angle_min_deg").value)
        self._talk_turn_angle_max = max(self._talk_turn_angle_min, self.get_parameter("talk_turn_angle_max_deg").value)
        self._talk_move_speed = abs(self.get_parameter("talk_move_speed_m_s").value)
        self._talk_turn_speed = abs(self.get_parameter("talk_turn_speed_rad_s").value)
        self._talk_move_interval = max(0.0, self.get_parameter("talk_move_interval_s").value)

        seed = self.get_parameter("random_seed").value
        if seed >= 0:
            random.seed(seed)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "/behavior/state", reliable_qos)

        # Subscribers
        self.create_subscription(Bool, "/tts/is_speaking", self._on_tts_speaking, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, best_effort_qos)

        # Marker pose subscribers are created dynamically per marker ID
        self._marker_subs: dict = {}
        self._marker_lock = threading.Lock()
        self._marker_poses: dict = {}  # marker_id -> (pose, timestamp)
        self._create_marker_subscriber(self._home_marker_id)

        # State
        self._state = self.STATE_DISABLED if not self._enabled else self.STATE_IDLE
        self._tts_speaking = False
        self._external_cmd_vel_active = False
        self._last_external_cmd_time = 0.0
        self._last_odom_time = 0.0
        self._odom_valid = False

        self._home_x = 0.0
        self._home_y = 0.0
        self._home_theta = 0.0
        self._home_locked = False

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_theta = 0.0

        self._anim_start_time = 0.0
        self._anim_linear = 0.0
        self._anim_angular = 0.0

        # Track our own cmd_vel so we don't treat it as an external command
        self._last_published_linear = 0.0
        self._last_published_angular = 0.0

        # Timestamps
        self._last_idle_time = time.time()
        self._last_state_pub = 0.0

        # Control loop at 20 Hz
        self._control_timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            "AnimationNode initialized:\n"
            f"  enabled: {self._enabled}\n"
            f"  idle_timeout: {self._idle_timeout_s}s\n"
            f"  max_displacement: {self._max_displacement_m}m\n"
            f"  talk_sway: {self._talk_sway_speed}m/s @ {self._talk_sway_period}s\n"
            f"  talk_animation_mode: {self._talk_animation_mode}\n"
            f"  talk_move_distance: {self._talk_move_distance_min}-{self._talk_move_distance_max}m\n"
            f"  talk_turn_angle: {self._talk_turn_angle_min}-{self._talk_turn_angle_max}°\n"
            f"  home_marker_id: {self._home_marker_id}"
        )

    def _create_marker_subscriber(self, marker_id: int):
        """Create a subscriber for a specific marker pose topic."""
        topic = f"/behavior/marker_poses/marker_{marker_id}"
        if topic in self._marker_subs:
            return
        sub = self.create_subscription(
            PoseStamped, topic, lambda msg, mid=marker_id: self._on_marker_pose(mid, msg), 10
        )
        self._marker_subs[topic] = sub
        self.get_logger().info(f"Subscribed to marker pose: {topic}")

    def _on_tts_speaking(self, msg: Bool):
        self._tts_speaking = bool(msg.data)

    def _on_cmd_vel(self, msg: Twist):
        """Detect external commands. Animation pauses while external cmd_vel flows."""
        # Ignore our own cmd_vel echoes so we don't pause ourselves
        if (
            abs(msg.linear.x - self._last_published_linear) < 0.001
            and abs(msg.angular.z - self._last_published_angular) < 0.001
        ):
            return

        # Ignore very tiny values (noise / zero-velocity heartbeats)
        if abs(msg.linear.x) > 0.005 or abs(msg.angular.z) > 0.005:
            self._external_cmd_vel_active = True
            self._last_external_cmd_time = time.time()

    def _on_odom(self, msg: Odometry):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._current_theta = math.atan2(siny, cosy)
        self._last_odom_time = time.time()
        self._odom_valid = True

        if not self._home_locked:
            self._set_home(self._current_x, self._current_y, self._current_theta)

    def _on_marker_pose(self, marker_id: int, msg: PoseStamped):
        with self._marker_lock:
            self._marker_poses[marker_id] = (msg, time.time())

    def _set_home(self, x: float, y: float, theta: float):
        self._home_x = x
        self._home_y = y
        self._home_theta = theta
        self._home_locked = True
        self.get_logger().info(
            f"Home locked: x={x:.3f}, y={y:.3f}, theta={math.degrees(theta):.1f}°"
        )

    def _odom_fresh(self) -> bool:
        return self._odom_valid and (time.time() - self._last_odom_time) < self._odom_timeout

    def _external_active(self) -> bool:
        if self._external_cmd_vel_active:
            if time.time() - self._last_external_cmd_time > self._cmd_vel_timeout:
                self._external_cmd_vel_active = False
        return self._external_cmd_vel_active

    def _displacement(self) -> Tuple[float, float, float]:
        """Return (dx, dy, distance) from home in odometry frame."""
        dx = self._current_x - self._home_x
        dy = self._current_y - self._home_y
        return dx, dy, math.hypot(dx, dy)

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _angle_to_home(self) -> float:
        """Angle from current position to home point."""
        dx = self._home_x - self._current_x
        dy = self._home_y - self._current_y
        return math.atan2(dy, dx)

    def _publish_state(self):
        now = time.time()
        if now - self._last_state_pub < 0.2:
            return
        self._last_state_pub = now
        msg = String()
        msg.data = self._state
        self._state_pub.publish(msg)

    def _publish_cmd_vel(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._last_published_linear = msg.linear.x
        self._last_published_angular = msg.angular.z
        self._cmd_vel_pub.publish(msg)

    def _stop(self):
        self._publish_cmd_vel(0.0, 0.0)

    def _select_random_move(self):
        """Pick a small random movement."""
        moves = [
            (self._anim_linear_speed, 0.0),    # forward
            (-self._anim_linear_speed, 0.0),   # backward
            (0.0, self._anim_angular_speed),   # turn left
            (0.0, -self._anim_angular_speed),  # turn right
            (self._anim_linear_speed * 0.7, self._anim_angular_speed * 0.5),
            (self._anim_linear_speed * 0.7, -self._anim_angular_speed * 0.5),
        ]
        self._anim_linear, self._anim_angular = random.choice(moves)
        self._anim_start_time = time.time()
        self.get_logger().info(
            f"Animation move: linear={self._anim_linear:.3f}, "
            f"angular={math.degrees(self._anim_angular):.1f}°/s"
        )

    def _select_talk_move(self):
        """Pick a small random movement while talking (2-6 in / small turns)."""
        move_type = random.choice([
            "forward", "backward", "turn_left", "turn_right"
        ])

        distance = random.uniform(self._talk_move_distance_min, self._talk_move_distance_max)
        angle_deg = random.uniform(self._talk_turn_angle_min, self._talk_turn_angle_max)

        if move_type == "forward":
            self._anim_linear = self._talk_move_speed
            self._anim_angular = 0.0
            self._anim_duration = distance / self._talk_move_speed
        elif move_type == "backward":
            self._anim_linear = -self._talk_move_speed
            self._anim_angular = 0.0
            self._anim_duration = distance / self._talk_move_speed
        elif move_type == "turn_left":
            self._anim_linear = 0.0
            self._anim_angular = self._talk_turn_speed
            self._anim_duration = math.radians(angle_deg) / self._talk_turn_speed
        else:  # turn_right
            self._anim_linear = 0.0
            self._anim_angular = -self._talk_turn_speed
            self._anim_duration = math.radians(angle_deg) / self._talk_turn_speed

        self._anim_start_time = time.time()
        self.get_logger().info(
            f"Talk move ({move_type}): dist={distance:.3f}m, "
            f"angle={angle_deg:.1f}°, duration={self._anim_duration:.2f}s"
        )

    def _talking_velocity(self, now: float) -> Tuple[float, float]:
        """Gentle sinusoidal forward/back sway."""
        phase = 2.0 * math.pi * now / self._talk_sway_period
        linear = self._talk_sway_speed * math.sin(phase)
        # Tiny superimposed rotation so it feels a bit more alive
        angular = 0.3 * self._anim_angular_speed * math.sin(phase * 0.7)
        return linear, angular

    def _marker_home_error(self) -> Optional[Tuple[float, float, float]]:
        """Return (x_err, y_err, yaw_err) relative to recorded home marker pose.

        For v1 this is a placeholder for visual-servo homing. The node already
        records and compares marker poses; a future iteration can command based
        on this error directly.
        """
        with self._marker_lock:
            home_pose = self._marker_poses.get(self._home_marker_id)
        if home_pose is None:
            return None
        pose, t = home_pose
        if time.time() - t > self._marker_visible_timeout:
            return None
        # At startup we would record a reference; here we just report current offset
        return (pose.pose.position.x, pose.pose.position.y, 0.0)

    def _return_home_velocity(self) -> Tuple[float, float]:
        """Command velocities to drive back to home pose using odometry."""
        dx, dy, dist = self._displacement()
        heading_err = self._normalize_angle(self._home_theta - self._current_theta)

        if dist > 0.02:
            # Far from home position: turn toward home point, then drive
            target_angle = self._angle_to_home()
            angle_error = self._normalize_angle(target_angle - self._current_theta)
            if abs(angle_error) > math.radians(15.0):
                angular = self._return_angular_speed * math.copysign(1.0, angle_error)
                return 0.0, angular
            linear = min(self._return_linear_speed, dist)
            return linear, 0.0

        # Close to home position: fix final heading
        if abs(heading_err) > math.radians(10.0):
            angular = self._return_angular_speed * math.copysign(1.0, heading_err)
            return 0.0, angular

        return 0.0, 0.0

    def _control_loop(self):
        if not self._enabled:
            if self._state != self.STATE_DISABLED:
                self._stop()
                self._state = self.STATE_DISABLED
                self._publish_state()
            return

        if not self._odom_fresh():
            self._stop()
            self._state = self.STATE_IDLE
            self._publish_state()
            return

        self._publish_state()

        # Safety: if displacement exceeded, force return home
        _, _, dist = self._displacement()
        if dist > self._max_displacement_m and self._state != self.STATE_RETURNING_HOME:
            self.get_logger().warning(
                f"Displacement {dist:.3f}m exceeds limit {self._max_displacement_m}m; returning home"
            )
            self._state = self.STATE_RETURNING_HOME

        # External motion always pauses animation. Do NOT publish cmd_vel here so
        # we never fight the external controller.
        if self._external_active():
            if self._state not in (self.STATE_IDLE, self.STATE_RETURNING_HOME):
                self.get_logger().info("External cmd_vel detected; pausing animation")
                self._state = self.STATE_IDLE
            self._last_idle_time = time.time()
            return

        if self._state == self.STATE_IDLE:
            self._stop()
            # Talking takes precedence over idle
            if self._tts_speaking:
                self._state = self.STATE_TALKING
                return
            # Random idle move
            if time.time() - self._last_idle_time > self._idle_timeout_s:
                self._select_random_move()
                self._state = self.STATE_ANIMATING
            return

        if self._state == self.STATE_TALKING:
            if not self._tts_speaking:
                self._state = self.STATE_IDLE
                self._last_idle_time = time.time()
                self._stop()
                return
            if self._talk_animation_mode == "sway":
                linear, angular = self._talking_velocity(time.time())
                self._publish_cmd_vel(linear, angular)
                return

            # Random small moves while talking
            elapsed = time.time() - self._anim_start_time
            if elapsed > self._anim_duration + self._talk_move_interval:
                self._select_talk_move()
            elif elapsed > self._anim_duration:
                self._publish_cmd_vel(0.0, 0.0)
            else:
                self._publish_cmd_vel(self._anim_linear, self._anim_angular)
            return

        if self._state == self.STATE_ANIMATING:
            # Check if we drifted too far during the animation
            if dist > self._max_displacement_m:
                self._state = self.STATE_RETURNING_HOME
                return
            if time.time() - self._anim_start_time > self._anim_duration:
                self._state = self.STATE_IDLE
                self._last_idle_time = time.time()
                self._stop()
                return
            self._publish_cmd_vel(self._anim_linear, self._anim_angular)
            return

        if self._state == self.STATE_RETURNING_HOME:
            _, _, dist = self._displacement()
            heading_err = abs(self._normalize_angle(self._home_theta - self._current_theta))
            if dist < 0.02 and heading_err < math.radians(10.0):
                self.get_logger().info("Returned home")
                self._state = self.STATE_IDLE
                self._last_idle_time = time.time()
                self._stop()
                return
            linear, angular = self._return_home_velocity()
            self._publish_cmd_vel(linear, angular)
            return

    def destroy_node(self):
        self.get_logger().info("Shutting down AnimationNode...")
        self._stop()
        if hasattr(self, "_control_timer"):
            self._control_timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AnimationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
