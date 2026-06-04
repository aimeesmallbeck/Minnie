#!/usr/bin/env python3
"""Test heading alignment (point-base) behavior for AimeeNav.

Publishes goals at various angles and verifies the robot rotates in place
until aligned before driving forward.

Usage (with robot stack running):
    ros2 run aimee_nav test_point_base.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


class PointBaseTest(Node):
    def __init__(self):
        super().__init__('point_base_test')
        self._best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, self._best_effort_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, self._best_effort_qos)

        self._odom_yaw = None
        self._last_cmd = Twist()

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def _cmd_vel_cb(self, msg):
        self._last_cmd = msg

    def wait_for_odom(self, timeout=5.0):
        self.get_logger().info("Waiting for /odom...")
        start = time.time()
        while self._odom_yaw is None and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._odom_yaw is None:
            self.get_logger().error("No odometry received! Is AimeeNav running?")
            return False
        return True

    def publish_goal(self, x, y, yaw_deg):
        """Publish a goal pose."""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        yaw = math.radians(yaw_deg)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pub.publish(msg)
        self.get_logger().info(f"Goal: ({x:.2f}, {y:.2f}) @ {yaw_deg:.0f}°")

    def clear_goal(self):
        """Publish a goal at current pose to effectively stop."""
        if self._odom_yaw is not None:
            self.publish_goal(0.0, 0.0, math.degrees(self._odom_yaw))

    def monitor_alignment(self, target_yaw, timeout=15.0, tol_deg=8.5):
        """Monitor until heading is aligned and robot starts driving.

        Returns a dict with timing and accuracy metrics.
        """
        start_t = time.time()
        aligned_t = None
        driving_t = None
        max_angular = 0.0
        samples = 0

        while time.time() - start_t < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            samples += 1

            if self._odom_yaw is not None:
                err = self._odom_yaw - target_yaw
                while err > math.pi:
                    err -= 2.0 * math.pi
                while err < -math.pi:
                    err += 2.0 * math.pi

                if aligned_t is None and abs(err) < math.radians(tol_deg):
                    aligned_t = time.time() - start_t
                    self.get_logger().info(
                        f"  Aligned in {aligned_t:.1f}s (err={math.degrees(err):.1f}°)"
                    )

                if driving_t is None and self._last_cmd.linear.x > 0.03:
                    driving_t = time.time() - start_t
                    self.get_logger().info(
                        f"  Started driving in {driving_t:.1f}s (vx={self._last_cmd.linear.x:.3f})"
                    )

            if abs(self._last_cmd.angular.z) > max_angular:
                max_angular = abs(self._last_cmd.angular.z)

            # Stop monitoring once both aligned and driving
            if aligned_t is not None and driving_t is not None:
                break

        # Final reading
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        final_err_deg = 0.0
        if self._odom_yaw is not None:
            final_err_deg = math.degrees(self._odom_yaw - target_yaw)
            while final_err_deg > 180:
                final_err_deg -= 360
            while final_err_deg < -180:
                final_err_deg += 360

        return {
            'aligned_t': aligned_t,
            'driving_t': driving_t,
            'max_angular': max_angular,
            'final_err_deg': final_err_deg,
            'samples': samples,
        }

    def test_case(self, name, gx, gy, goal_yaw_deg):
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"TEST: {name}")
        self.get_logger().info(f"{'='*50}")

        if not self.wait_for_odom():
            return

        start_yaw = self._odom_yaw
        target_yaw = math.atan2(gy, gx)

        self.get_logger().info(
            f"Start yaw: {math.degrees(start_yaw):.1f}° | "
            f"Expected target heading: {math.degrees(target_yaw):.1f}°"
        )

        self.publish_goal(gx, gy, goal_yaw_deg)
        result = self.monitor_alignment(target_yaw, timeout=15.0)

        # Evaluate
        passed = True
        if result['aligned_t'] is None:
            self.get_logger().warn("  FAIL: Never aligned within tolerance")
            passed = False
        else:
            self.get_logger().info(f"  Align time: {result['aligned_t']:.1f}s")

        if result['driving_t'] is None:
            self.get_logger().warn("  FAIL: Never started driving")
            passed = False
        elif result['aligned_t'] is not None and result['driving_t'] < result['aligned_t']:
            self.get_logger().warn(
                f"  FAIL: Started driving ({result['driving_t']:.1f}s) BEFORE "
                f"alignment ({result['aligned_t']:.1f}s)"
            )
            passed = False
        else:
            self.get_logger().info(f"  Drive time: {result['driving_t']:.1f}s")

        self.get_logger().info(f"  Max angular: {result['max_angular']:.2f} rad/s")
        self.get_logger().info(f"  Final heading error: {result['final_err_deg']:.1f}°")

        if passed:
            self.get_logger().info("  RESULT: PASS")
        else:
            self.get_logger().info("  RESULT: FAIL")

        # Clear goal by publishing at current location
        self.clear_goal()
        time.sleep(1.0)
        return passed

    def run_all(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("POINT BASE (HEADING ALIGNMENT) TEST")
        self.get_logger().info("=" * 50)
        self.get_logger().info(
            "Ensure robot has ~1m clear space in all directions."
        )
        time.sleep(2.0)

        results = []

        # Test 1: Straight ahead — should drive with minimal turning
        results.append(self.test_case("Straight ahead", 1.0, 0.0, 0))

        # Test 2: 90° left — should turn in place, then drive
        results.append(self.test_case("90° left", 0.0, 1.0, 90))

        # Test 3: 90° right
        results.append(self.test_case("90° right", 0.0, -1.0, -90))

        # Test 4: 180° behind
        results.append(self.test_case("180° behind", -1.0, 0.0, 180))

        self.get_logger().info(f"\n{'='*50}")
        passed = sum(results)
        total = len(results)
        self.get_logger().info(f"RESULTS: {passed}/{total} tests passed")
        self.get_logger().info(f"{'='*50}")


def main():
    rclpy.init()
    node = PointBaseTest()
    try:
        node.run_all()
    except KeyboardInterrupt:
        pass
    finally:
        node.clear_goal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
