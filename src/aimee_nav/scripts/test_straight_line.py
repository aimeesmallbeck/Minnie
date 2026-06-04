#!/usr/bin/env python3
"""
Straight-line goal test for AimeeNav.

Publishes a goal 1 meter directly ahead of the robot's current position,
monitors /odom progress, and reports lateral error (left/right drift).

Usage:
    ros2 run aimee_nav test_straight_line.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class StraightLineTest(Node):
    def __init__(self):
        super().__init__('straight_line_test')

        self.declare_parameter('distance', 1.0)       # meters ahead
        self.declare_parameter('timeout', 30.0)       # seconds
        self.declare_parameter('goal_tolerance', 0.20)  # meters

        self._distance = self.get_parameter('distance').value
        self._timeout = self.get_parameter('timeout').value
        self._goal_tolerance = self.get_parameter('goal_tolerance').value

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
        self._start_x = 0.0
        self._start_y = 0.0
        self._start_yaw = 0.0

        self.create_subscription(Odometry, '/odom', self._odom_cb, self._best_effort_qos)

    def _odom_cb(self, msg):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def wait_for_odom(self, timeout=20.0):
        self.get_logger().info("Waiting for /odom...")
        start = time.time()
        while self._odom_yaw is None and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._odom_yaw is None:
            self.get_logger().error("No odometry received — is AimeeNav running?")
            return False
        self.get_logger().info(
            f"Odometry ready. Start pose: x={self._odom_x:.2f}, y={self._odom_y:.2f}, "
            f"yaw={math.degrees(self._odom_yaw):.1f}°"
        )
        return True

    def run_test(self):
        if not self.wait_for_odom():
            return

        # Record start pose
        self._start_x = self._odom_x
        self._start_y = self._odom_y
        self._start_yaw = self._odom_yaw

        # Goal = distance meters straight ahead
        goal_x = self._start_x + self._distance * math.cos(self._start_yaw)
        goal_y = self._start_y + self._distance * math.sin(self._start_yaw)

        qz = math.sin(self._start_yaw / 2.0)
        qw = math.cos(self._start_yaw / 2.0)

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(goal_x)
        msg.pose.position.y = float(goal_y)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        self.get_logger().info(
            f"\n=== STRAIGHT-LINE TEST ==="
            f"\nGoal:       x={goal_x:.2f}, y={goal_y:.2f}"
            f"\nStart yaw:  {math.degrees(self._start_yaw):.1f}°"
            f"\nTimeout:    {self._timeout:.0f}s"
            f"\n=========================="
        )

        time.sleep(2.0)
        self.get_logger().info("SENDING GOAL NOW — robot will move forward 1 meter!")
        self._goal_pub.publish(msg)

        # Monitor progress
        start_t = time.time()
        last_print = start_t
        reached = False

        while time.time() - start_t < self._timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self._odom_yaw is None:
                continue

            dx = goal_x - self._odom_x
            dy = goal_y - self._odom_y
            dist_to_goal = math.hypot(dx, dy)

            # Lateral error = perpendicular distance from straight line
            # Project (current - start) onto perpendicular of start_yaw
            px = self._odom_x - self._start_x
            py = self._odom_y - self._start_y
            # Perp vector: (-sin(yaw), cos(yaw))
            lateral_error = px * (-math.sin(self._start_yaw)) + py * math.cos(self._start_yaw)

            # Forward progress along start direction
            forward_progress = px * math.cos(self._start_yaw) + py * math.sin(self._start_yaw)

            now = time.time()
            if now - last_print >= 1.0:
                self.get_logger().info(
                    f"  progress={forward_progress:5.2f}m  "
                    f"remaining={dist_to_goal:5.2f}m  "
                    f"lateral_err={lateral_error:+.3f}m  "
                    f"yaw={math.degrees(self._odom_yaw):6.1f}°"
                )
                last_print = now

            if dist_to_goal < self._goal_tolerance and not reached:
                self.get_logger().info(
                    f"\n>>> GOAL REACHED (odom) after {now - start_t:.1f}s <<<")
                reached = True

            if reached and dist_to_goal < self._goal_tolerance:
                # Stay for one more second then break
                if now - start_t > 2.0:
                    break

        # Final report
        final_dx = goal_x - self._odom_x
        final_dy = goal_y - self._odom_y
        final_dist = math.hypot(final_dx, final_dy)

        px = self._odom_x - self._start_x
        py = self._odom_y - self._start_y
        final_lateral = px * (-math.sin(self._start_yaw)) + py * math.cos(self._start_yaw)
        final_forward = px * math.cos(self._start_yaw) + py * math.sin(self._start_yaw)

        self.get_logger().info(
            f"\n=== FINAL REPORT ==="
            f"\nGoal:            ({goal_x:.2f}, {goal_y:.2f})"
            f"\nFinal odom:      ({self._odom_x:.2f}, {self._odom_y:.2f})"
            f"\nForward progress: {final_forward:.3f}m  (target: {self._distance:.1f}m)"
            f"\nLateral error:    {final_lateral:+.3f}m  (positive = left of line)"
            f"\nDistance to goal: {final_dist:.3f}m"
            f"\nFinal yaw:        {math.degrees(self._odom_yaw):.1f}°"
            f"\nGoal reached:     {reached}"
        )

        # Stop
        self._cmd_pub.publish(Twist())


def main():
    rclpy.init(args=sys.argv)
    node = StraightLineTest()
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
