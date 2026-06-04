#!/usr/bin/env python3
"""
90-degree turn test for AimeeNav with IMU yaw fusion.

Publishes a goal 1 meter to the left (90° CCW) or right (90° CW),
monitors heading progress via /odom, and reports turn accuracy.

Usage:
    # Terminal 1: launch AimeeNav
    ros2 launch aimee_nav aimee_nav.launch.py

    # Terminal 2: run this test
    ros2 run aimee_nav test_90deg_turn.py --ros-args -p direction:=left
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class TurnTest(Node):
    def __init__(self):
        super().__init__('turn_test')

        self.declare_parameter('direction', 'left')   # 'left' or 'right'
        self.declare_parameter('distance', 1.0)       # meters to goal
        self.declare_parameter('timeout', 30.0)       # seconds

        self._direction = self.get_parameter('direction').value
        self._distance = self.get_parameter('distance').value
        self._timeout = self.get_parameter('timeout').value

        self._best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = None
        self._imu_yaw = None
        self._goal_active = False

        self.create_subscription(Odometry, '/odom', self._odom_cb, self._best_effort_qos)
        self.create_subscription(Imu, '/imu', self._imu_cb, self._best_effort_qos)

    def _odom_cb(self, msg):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def _imu_cb(self, msg):
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._imu_yaw = math.atan2(siny, cosy)

    def wait_for_odom(self, timeout=20.0):
        self.get_logger().info("Waiting for /odom...")
        start = time.time()
        last_print = start
        while self._odom_yaw is None and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()
            if now - last_print >= 3.0:
                self.get_logger().info("Still waiting for /odom...")
                last_print = now
        if self._odom_yaw is None:
            self.get_logger().error("No odometry received — is AimeeNav running?")
            return False
        self.get_logger().info(f"Odometry ready. Start yaw = {math.degrees(self._odom_yaw):.1f}°")
        return True

    def run_test(self):
        if not self.wait_for_odom():
            return

        sign = 1.0 if self._direction == 'left' else -1.0
        delta_rad = sign * math.pi / 2.0
        target_yaw = self._odom_yaw + delta_rad

        # Normalize target
        while target_yaw > math.pi:
            target_yaw -= 2.0 * math.pi
        while target_yaw < -math.pi:
            target_yaw += 2.0 * math.pi

        # Build goal: distance meters from current position, at target heading
        goal_x = self._odom_x + self._distance * math.cos(target_yaw)
        goal_y = self._odom_y + self._distance * math.sin(target_yaw)

        qz = math.sin(target_yaw / 2.0)
        qw = math.cos(target_yaw / 2.0)

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(goal_x)
        msg.pose.position.y = float(goal_y)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        self.get_logger().info(
            f"\n=== 90° TURN TEST ==="
            f"\nDirection:  {self._direction.upper()} ({sign*90:.0f}°)"
            f"\nGoal:       x={goal_x:.2f}, y={goal_y:.2f}"
            f"\nTarget yaw: {math.degrees(target_yaw):.1f}°"
            f"\nTimeout:    {self._timeout:.0f}s"
        )

        # Wait a moment for user to read
        time.sleep(2.0)
        self.get_logger().info("SENDING GOAL NOW — robot will move!")
        self._goal_pub.publish(msg)
        self._goal_active = True

        # Monitor
        start_t = time.time()
        last_print = start_t
        max_yaw_reached = 0.0
        settled = False

        while time.time() - start_t < self._timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self._odom_yaw is None:
                continue

            err = target_yaw - self._odom_yaw
            while err > math.pi:
                err -= 2.0 * math.pi
            while err < -math.pi:
                err += 2.0 * math.pi

            progress = abs(delta_rad - err)
            if progress > abs(max_yaw_reached):
                max_yaw_reached = progress

            now = time.time()
            if now - last_print >= 1.0:
                odom_deg = math.degrees(self._odom_yaw) if self._odom_yaw is not None else 0.0
                imu_deg = math.degrees(self._imu_yaw) if self._imu_yaw is not None else 0.0
                self.get_logger().info(
                    f"  odom={odom_deg:6.1f}°  imu={imu_deg:6.1f}°  "
                    f"err={math.degrees(err):5.1f}°  progress={math.degrees(progress):5.1f}°"
                )
                last_print = now

            if abs(err) < math.radians(5.0) and not settled:
                self.get_logger().info(
                    f"\n>>> SETTLED within 5° after {now - start_t:.1f}s <<<"
                )
                settled = True

            if abs(err) < math.radians(2.0) and settled:
                self.get_logger().info(
                    f">>> FINAL ALIGNMENT within 2° after {now - start_t:.1f}s <<<"
                )
                break
        else:
            self.get_logger().warn("\n>>> TIMEOUT — turn did not complete <<<")

        final_err = target_yaw - self._odom_yaw
        while final_err > math.pi:
            final_err -= 2.0 * math.pi
        while final_err < -math.pi:
            final_err += 2.0 * math.pi

        self.get_logger().info(
            f"\n=== RESULTS ==="
            f"\nTarget yaw:        {math.degrees(target_yaw):.1f}°"
            f"\nFinal odom yaw:    {math.degrees(self._odom_yaw):.1f}°"
            f"\nHeading error:     {math.degrees(final_err):.1f}°"
            f"\nMax progress:      {math.degrees(max_yaw_reached):.1f}°"
            f"\nSettled:           {settled}"
        )

        # Cancel goal by sending a stop / zero-velocity command
        self._cmd_pub.publish(Twist())


def main():
    rclpy.init(args=sys.argv)
    node = TurnTest()
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
