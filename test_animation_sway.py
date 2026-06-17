#!/usr/bin/env python3
"""Gentle talking-sway validation script for aimee_behaviors/animation_node."""

import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class AnimationSwayTest(Node):
    def __init__(self):
        super().__init__("animation_sway_test")

        self._cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, odom_qos)
        self._tts_pub = self.create_publisher(Bool, "/tts/is_speaking", 10)

        self._lock = threading.Lock()
        self._max_linear = 0.0
        self._max_angular = 0.0
        self._cmd_count = 0
        self._start_x = None
        self._start_y = None
        self._max_displacement = 0.0
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_theta = 0.0

    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            self._cmd_count += 1
            self._max_linear = max(self._max_linear, abs(msg.linear.x))
            self._max_angular = max(self._max_angular, abs(msg.angular.z))

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)

        with self._lock:
            if self._start_x is None:
                self._start_x = x
                self._start_y = y
            self._last_x = x
            self._last_y = y
            self._last_theta = theta
            disp = math.hypot(x - self._start_x, y - self._start_y)
            self._max_displacement = max(self._max_displacement, disp)

    def set_tts(self, speaking: bool):
        msg = Bool()
        msg.data = bool(speaking)
        self._tts_pub.publish(msg)
        self.get_logger().info(f"Published /tts/is_speaking={speaking}")

    def summary(self):
        with self._lock:
            return {
                "cmd_count": self._cmd_count,
                "max_linear_m_s": self._max_linear,
                "max_angular_rad_s": self._max_angular,
                "max_displacement_m": self._max_displacement,
                "final_x": self._last_x,
                "final_y": self._last_y,
                "final_theta_deg": math.degrees(self._last_theta),
            }


def main():
    rclpy.init()
    node = AnimationSwayTest()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Let subscriptions connect and record a baseline odom sample
    time.sleep(1.0)
    node.get_logger().info("Starting talking-sway test: /tts/is_speaking=true for 6s")
    node.set_tts(True)

    time.sleep(6.0)

    node.set_tts(False)
    # Give animation_node one more cycle to stop and publish zero velocity
    time.sleep(0.5)

    executor.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()

    s = node.summary()
    print("\n=== Talking-sway test results ===")
    print(f"/cmd_vel messages seen: {s['cmd_count']}")
    print(f"Max linear speed:    {s['max_linear_m_s']:.4f} m/s")
    print(f"Max angular speed:   {s['max_angular_rad_s']:.4f} rad/s")
    print(f"Max displacement:    {s['max_displacement_m']:.4f} m ({s['max_displacement_m']*1000:.1f} mm)")
    print(f"Final pose:          x={s['final_x']:.4f}, y={s['final_y']:.4f}, theta={s['final_theta_deg']:.2f}°")

    limits = {
        "linear_limit": 0.015,
        "angular_limit": 0.024,  # 0.3 * 0.08
        "displacement_limit": 0.06,
    }
    ok = (
        s["max_linear_m_s"] <= limits["linear_limit"] + 1e-4
        and s["max_angular_rad_s"] <= limits["angular_limit"] + 1e-4
        and s["max_displacement_m"] <= limits["displacement_limit"]
    )
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
