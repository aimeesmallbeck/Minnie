#!/usr/bin/env python3
"""
Sensor validation script for AimeeNav.

Subscribe to /imu and /odom and print live heading values.
Use this while physically rotating the robot by hand to verify:
  - IMU yaw responds correctly to rotation
  - EKF heading tracks IMU + scan matching
  - Gyro sign is correct

Usage:
    # Terminal 1: launch AimeeNav
    ros2 launch aimee_nav aimee_nav.launch.py

    # Terminal 2: run this script
    ros2 run aimee_nav validate_sensors.py

Then pick up the robot and rotate it slowly in 90° increments.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        self._best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._odom_yaw = None
        self._imu_yaw = None
        self._imu_gyro_z = None
        self._last_imu_yaw = None
        self._last_imu_time = None

        self.create_subscription(Odometry, '/odom', self._odom_cb, self._best_effort_qos)
        self.create_subscription(Imu, '/imu', self._imu_cb, self._best_effort_qos)

        # Publish zero cmd_vel to prevent any auto-movement
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._timer = self.create_timer(0.5, self._send_stop)

    def _send_stop(self):
        self._cmd_pub.publish(Twist())

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    def _imu_cb(self, msg):
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self._imu_yaw = yaw
        self._imu_gyro_z = msg.angular_velocity.z

    def run(self):
        print("\n" + "=" * 65)
        print(" SENSOR VALIDATION")
        print(" Pick up the robot and rotate it slowly by hand.")
        print(" All values in degrees.  Press Ctrl+C to stop.")
        print("=" * 65 + "\n")
        print(f"{'Time':>6}  {'Odom':>8}  {'IMU Yaw':>8}  {'IMU Gyro':>9}  {'Delta IMU':>10}")
        print("-" * 65)

        start = time.time()
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                now = time.time()

                if self._imu_yaw is not None:
                    delta = 0.0
                    if self._last_imu_yaw is not None:
                        delta = math.degrees(self._imu_yaw - self._last_imu_yaw)
                        # Normalize to [-180, 180]
                        while delta > 180:
                            delta -= 360
                        while delta < -180:
                            delta += 360
                    self._last_imu_yaw = self._imu_yaw

                    odom_str = f"{math.degrees(self._odom_yaw):7.1f}" if self._odom_yaw is not None else "   ---  "
                    gyro_str = f"{math.degrees(self._imu_gyro_z):7.1f}" if self._imu_gyro_z is not None else "   ---  "

                    print(f"{now - start:6.1f}  {odom_str}  {math.degrees(self._imu_yaw):7.1f}  {gyro_str}°/s  {delta:+7.1f}°")
        except KeyboardInterrupt:
            pass
        finally:
            self._cmd_pub.publish(Twist())


def main():
    rclpy.init(args=sys.argv)
    node = SensorValidator()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
