#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL
#
# SPDX-License-Identifier: MPL-2.0

"""
AimeeNav — Integrated Self-Contained Navigation Node.

A single ROS2 node that directly interfaces with the LD19 lidar and
Wave Rover base, performs local mapping, path planning, and obstacle
avoidance, all in-process without DDS dependencies for the navigation
loop.

Publishers (for visualization / interoperability):
    /scan           sensor_msgs/LaserScan
    /map            nav_msgs/OccupancyGrid
    /odom           nav_msgs/Odometry
    /tf             geometry_msgs/TransformStamped
    /path           nav_msgs/Path
    /cmd_vel        geometry_msgs/Twist

Subscribers:
    /goal_pose      geometry_msgs/PoseStamped

Action Server:
    navigate_to_pose  (not yet implemented — stretch goal)

Usage:
    ros2 run aimee_nav aimee_nav_node
    ros2 launch aimee_nav aimee_nav.launch.py
"""

import base64
import json
import math
import os
import random
import threading
import time
from pathlib import Path
from typing import Optional, Tuple, List, Dict, Any

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Imu, MagneticField
import tf2_ros
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool

from aimee_nav.ld19_driver import LD19Driver, LD19Scan
from aimee_nav.wave_rover_driver import WaveRoverDriver
from aimee_nav.yahboom_imu_driver import YahboomIMUDriver
from aimee_nav.local_grid_map_cpp import LocalGridMapCpp as LocalGridMap
from aimee_nav._core import GridMap, ScanMatcher, EKF2D, GlobalPlanner, PoseGraph, Keyframe, DWALocalPlanner, DWAConfig, MCL2D, FrontierDetector
from aimee_nav.simple_planner import SimplePlanner
from aimee_nav.obstacle_avoidance import ObstacleAvoidance
from aimee_nav.pid_controller import HeadingVelocityController
from aimee_nav.map_manager import MapManager
from aimee_nav.explore_engine import ExploreEngine


class AimeeNavNode(Node):
    """
    Integrated navigation node for AIMEE Robot.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__('aimee_nav', **kwargs)

        # ─── Parameters ───
        self.declare_parameters(namespace='', parameters=[
            # Lidar
            ('lidar_port', '/dev/ttyUSB1'),
            ('lidar_baud', 230400),
            ('lidar_frame_id', 'base_laser'),
            ('lidar_angle_offset_deg', 0.0),
            ('lidar_distance_scale', 1.0),

            # Rover
            ('rover_port', '/dev/ttyUSB0'),
            ('rover_baud', 115200),
            ('rover_http_ip', ''),
            ('wheel_separation', 0.172),
            ('wheel_radius', 0.04),
            ('max_speed', 0.5),
            ('control_mode', 'wheel_speed'),
            ('angular_scale', 1.0),
            ('accel_limit_linear', 0.0),
            ('accel_limit_angular', 0.0),

            # Grid map
            ('grid_size_m', 5.0),
            ('grid_resolution_m', 0.05),
            ('global_map_size_m', 20.0),
            ('global_map_resolution_m', 0.05),
            ('obstacle_inflation_m', 0.15),
            ('decay_time_s', 10.0),

            # Navigation
            ('safety_distance_m', 0.35),
            ('goal_tolerance_m', 0.25),
            ('replan_interval_s', 1.0),
            ('min_clearance_m', 0.20),
            ('navigation_mode', 'reactive'),  # 'reactive' or 'planned'

            # PID
            ('heading_kp', 2.0),
            ('heading_ki', 0.0),
            ('heading_kd', 0.5),
            ('velocity_kp', 1.0),
            ('velocity_ki', 0.0),
            ('velocity_kd', 0.0),

            # Behavior
            ('enable_reactive', True),
            ('enable_planning', True),
            ('enable_exploration', False),
            ('exploration_speed_scale', 0.3),
            ('min_frontier_size', 5),
            ('emergency_reverse_time_s', 0.5),

            # Frames
            ('map_frame', 'map'),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_link'),
            ('publish_tf', True),
            ('publish_map_odom_tf', True),

            # Timing
            ('nav_rate_hz', 10.0),
            ('publish_decimation', 10),  # Publish scan/map every N cycles
            ('lidar_downsample', 6),  # Use every Nth lidar point (1=full, 6=60 pts)
            ('scan_match_interval', 0.2),  # Seconds between scan matches
            ('scan_match_search_angle_rad', 0.5),  # Angular search window for scan matcher
            ('scan_match_score_threshold', 15.0),  # Min score to accept scan match correction
            ('heading_alignment_tolerance_rad', 0.15),  # Align heading before driving (rad)
            ('map_save_dir', '~/aimee_maps'),
            ('map_library_dir', '~/aimee_maps'),
            ('waypoints_file', ''),
            ('localization_mode', False),
            ('auto_save_on_complete', True),

            # MCL
            ('mcl_particles_max', 2000),
            ('mcl_particles_min', 250),
            ('mcl_kld_epsilon', 0.05),
            ('mcl_motion_noise_alpha1', 0.2),
            ('mcl_motion_noise_alpha2', 0.2),
            ('mcl_motion_noise_alpha3', 0.2),
            ('mcl_motion_noise_alpha4', 0.2),

            # Enhanced Exploration
            ('exploration_safety_margin_m', 0.40),
            ('exploration_info_gain_radius_m', 2.0),
            ('exploration_min_frontier_size_m', 0.25),
            ('exploration_complete_timeout_s', 60.0),
            ('exploration_enable_bootstrap_spin', True),
            ('explore_weight_info', 1.0),
            ('explore_weight_distance', 0.5),
            ('explore_weight_safety', 0.8),
            ('explore_weight_alignment', 0.3),

            # Yahboom IMU
            ('imu_port', '/dev/ttyUSB2'),
            ('imu_baud', 115200),
            ('enable_imu_yaw', True),
            ('imu_yaw_variance', 0.005),
            ('imu_yaw_scale', -1.0),
            ('imu_yaw_offset_deg', 0.0),
        ])

        # Read parameters
        self._lidar_port = self.get_parameter('lidar_port').value
        self._lidar_baud = self.get_parameter('lidar_baud').value
        self._lidar_frame_id = self.get_parameter('lidar_frame_id').value
        self._lidar_angle_offset = self.get_parameter('lidar_angle_offset_deg').value
        self._lidar_distance_scale = self.get_parameter('lidar_distance_scale').value

        self._rover_port = self.get_parameter('rover_port').value
        self._rover_baud = self.get_parameter('rover_baud').value
        self._rover_http_ip = self.get_parameter('rover_http_ip').value
        self._wheel_sep = self.get_parameter('wheel_separation').value
        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._max_speed = self.get_parameter('max_speed').value
        self._control_mode = self.get_parameter('control_mode').value
        self._angular_scale = self.get_parameter('angular_scale').value
        self._accel_limit_linear = self.get_parameter('accel_limit_linear').value
        self._accel_limit_angular = self.get_parameter('accel_limit_angular').value

        self._grid_size = self.get_parameter('grid_size_m').value
        self._grid_res = self.get_parameter('grid_resolution_m').value
        self._inflation = self.get_parameter('obstacle_inflation_m').value
        self._decay_time = self.get_parameter('decay_time_s').value

        self._safety_distance = self.get_parameter('safety_distance_m').value
        self._goal_tolerance = self.get_parameter('goal_tolerance_m').value
        self._replan_interval = self.get_parameter('replan_interval_s').value
        self._min_clearance = self.get_parameter('min_clearance_m').value
        self._nav_mode = self.get_parameter('navigation_mode').value

        self._heading_kp = self.get_parameter('heading_kp').value
        self._heading_ki = self.get_parameter('heading_ki').value
        self._heading_kd = self.get_parameter('heading_kd').value
        self._velocity_kp = self.get_parameter('velocity_kp').value
        self._velocity_ki = self.get_parameter('velocity_ki').value
        self._velocity_kd = self.get_parameter('velocity_kd').value

        self._enable_reactive = self.get_parameter('enable_reactive').value
        self._enable_planning = self.get_parameter('enable_planning').value
        self._enable_exploration = self.get_parameter('enable_exploration').value
        self._exploration_speed_scale = self.get_parameter('exploration_speed_scale').value
        self._min_frontier_size = self.get_parameter('min_frontier_size').value
        # navigation_mode can override the individual booleans for convenience
        if self._nav_mode == 'reactive':
            self._enable_reactive = True
            self._enable_planning = False
        elif self._nav_mode == 'planned':
            self._enable_reactive = True
            self._enable_planning = True
        self._emergency_reverse_time = self.get_parameter('emergency_reverse_time_s').value

        self._map_frame = self.get_parameter('map_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._do_publish_tf = self.get_parameter('publish_tf').value
        self._do_publish_map_odom_tf = self.get_parameter('publish_map_odom_tf').value

        self._nav_rate = self.get_parameter('nav_rate_hz').value
        self._publish_decimation = self.get_parameter('publish_decimation').value
        self._lidar_downsample = max(1, self.get_parameter('lidar_downsample').value)
        self._scan_match_interval = self.get_parameter('scan_match_interval').value
        self._scan_match_search_angle = self.get_parameter('scan_match_search_angle_rad').value
        self._scan_match_score_threshold = self.get_parameter('scan_match_score_threshold').value
        self._heading_align_tol = self.get_parameter('heading_alignment_tolerance_rad').value
        self._map_save_dir = os.path.expanduser(self.get_parameter('map_save_dir').value)
        self._map_library_dir = os.path.expanduser(self.get_parameter('map_library_dir').value)
        self._waypoints_file = self.get_parameter('waypoints_file').value
        self._localization_mode = self.get_parameter('localization_mode').value
        self._auto_save_on_complete = self.get_parameter('auto_save_on_complete').value

        # MCL params
        self._mcl_particles_max = self.get_parameter('mcl_particles_max').value
        self._mcl_particles_min = self.get_parameter('mcl_particles_min').value
        self._mcl_kld_epsilon = self.get_parameter('mcl_kld_epsilon').value
        self._mcl_alpha1 = self.get_parameter('mcl_motion_noise_alpha1').value
        self._mcl_alpha2 = self.get_parameter('mcl_motion_noise_alpha2').value
        self._mcl_alpha3 = self.get_parameter('mcl_motion_noise_alpha3').value
        self._mcl_alpha4 = self.get_parameter('mcl_motion_noise_alpha4').value

        # Exploration params
        self._exploration_safety_margin = self.get_parameter('exploration_safety_margin_m').value
        self._exploration_info_gain_radius = self.get_parameter('exploration_info_gain_radius_m').value
        self._exploration_min_frontier_size_m = self.get_parameter('exploration_min_frontier_size_m').value
        self._exploration_complete_timeout = self.get_parameter('exploration_complete_timeout_s').value
        self._exploration_enable_bootstrap_spin = self.get_parameter('exploration_enable_bootstrap_spin').value
        self._explore_weight_info = self.get_parameter('explore_weight_info').value
        self._explore_weight_distance = self.get_parameter('explore_weight_distance').value
        self._explore_weight_safety = self.get_parameter('explore_weight_safety').value
        self._explore_weight_alignment = self.get_parameter('explore_weight_alignment').value

        # IMU params
        self._imu_port = self.get_parameter('imu_port').value
        self._imu_baud = self.get_parameter('imu_baud').value
        self._enable_imu_yaw = self.get_parameter('enable_imu_yaw').value
        self._imu_yaw_variance = self.get_parameter('imu_yaw_variance').value
        self._imu_yaw_scale = self.get_parameter('imu_yaw_scale').value
        self._imu_yaw_offset_deg = self.get_parameter('imu_yaw_offset_deg').value

        # ─── QoS Profiles ───
        self._best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ─── Publishers ───
        self._scan_pub = self.create_publisher(LaserScan, '/scan', self._best_effort_qos)
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', self._reliable_qos)
        self._local_map_pub = self.create_publisher(OccupancyGrid, '/local_map', self._reliable_qos)
        self._odom_pub = self.create_publisher(Odometry, '/odom', self._best_effort_qos)
        self._path_pub = self.create_publisher(Path, '/path', self._reliable_qos)
        self._local_plan_pub = self.create_publisher(Path, '/local_plan', self._reliable_qos)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', self._best_effort_qos)
        self._imu_pub = self.create_publisher(Imu, '/imu', self._best_effort_qos)
        self._mag_pub = self.create_publisher(MagneticField, '/magnetometer', self._best_effort_qos)

        # ─── TF ───
        if self._do_publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ─── Subscribers ───
        self._goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._on_goal_pose, 10
        )
        self._go_to_waypoint_sub = self.create_subscription(
            String, '/go_to_waypoint_name', self._on_go_to_waypoint_name, 10
        )
        self._location_name_sub = self.create_subscription(
            String, '/location_name', self._on_location_name, 10
        )
        self._exploration_cmd_sub = self.create_subscription(
            String, '/exploration_command', self._on_exploration_command, 10
        )

        # ─── Action Server ───
        self._nav_action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self._navigate_to_pose_callback,
            goal_callback=self._nav_goal_callback,
            cancel_callback=self._nav_cancel_callback,
        )

        # ─── Map Manager ───
        self._map_manager = MapManager(self._map_library_dir, logger=self.get_logger())

        # ─── Services ───
        self._save_map_srv = self.create_service(Empty, '/save_map', self._on_save_map)
        self._load_map_srv = self.create_service(Empty, '/load_map', self._on_load_map)
        self._set_localization_srv = self.create_service(SetBool, '/set_localization_mode', self._on_set_localization_mode)
        self._clear_costmap_srv = self.create_service(Empty, '/clear_costmap', self._on_clear_costmap)
        self._reinit_localization_srv = self.create_service(Empty, '/reinitialize_global_localization', self._on_reinit_localization)

        # New map management services
        from aimee_msgs.srv import SaveMap, LoadMap, ListMaps, DeleteMap, ImportMap, ExportMap
        self._save_map_mgr_srv = self.create_service(SaveMap, '/map_manager/save', self._on_map_mgr_save)
        self._load_map_mgr_srv = self.create_service(LoadMap, '/map_manager/load', self._on_map_mgr_load)
        self._list_maps_srv = self.create_service(ListMaps, '/map_manager/list', self._on_map_mgr_list)
        self._delete_map_srv = self.create_service(DeleteMap, '/map_manager/delete', self._on_map_mgr_delete)
        self._import_map_srv = self.create_service(ImportMap, '/map_manager/import', self._on_map_mgr_import)
        self._export_map_srv = self.create_service(ExportMap, '/map_manager/export', self._on_map_mgr_export)

        # ─── Sub-modules ───
        self._lidar = LD19Driver(
            port=self._lidar_port,
            baudrate=self._lidar_baud,
            angle_offset_deg=self._lidar_angle_offset,
            distance_scale=self._lidar_distance_scale,
            queue_size=2,
        )
        self._rover = WaveRoverDriver(
            port=self._rover_port,
            baudrate=self._rover_baud,
            http_ip=self._rover_http_ip,
            wheel_separation=self._wheel_sep,
            wheel_radius=self._wheel_radius,
            max_speed=self._max_speed,
            max_angular=0.4,
            angular_scale=self._angular_scale,
            control_mode=self._control_mode,
            accel_limit_linear=self._accel_limit_linear,
            accel_limit_angular=self._accel_limit_angular,
        )
        self._imu = YahboomIMUDriver(
            port=self._imu_port,
            baudrate=self._imu_baud,
        )
        self._grid = LocalGridMap(
            size_m=self._grid_size,
            resolution_m=self._grid_res,
            inflation_m=self._inflation,
            decay_time_s=self._decay_time,
        )
        self._planner = SimplePlanner(self._grid)
        self._global_planner = GlobalPlanner()
        self._dwa_cfg = DWAConfig()
        self._dwa_cfg.max_vel_x = float(self._max_speed)
        self._dwa_cfg.max_vel_theta = 1.5
        self._dwa_cfg.acc_lim_x = 1.0
        self._dwa_cfg.acc_lim_theta = 2.0
        self._dwa_cfg.sim_time = 1.5
        self._dwa_cfg.dt = 0.1
        self._dwa_cfg.vx_samples = 20
        self._dwa_cfg.vtheta_samples = 20
        self._dwa = DWALocalPlanner(self._dwa_cfg)
        self._avoidance = ObstacleAvoidance(
            safety_distance_m=self._safety_distance,
            min_clearance_m=self._min_clearance,
            emergency_reverse_time_s=self._emergency_reverse_time,
        )
        self._controller = HeadingVelocityController(
            heading_kp=self._heading_kp,
            heading_ki=self._heading_ki,
            heading_kd=self._heading_kd,
            velocity_kp=self._velocity_kp,
            velocity_ki=self._velocity_ki,
            velocity_kd=self._velocity_kd,
            max_linear=self._max_speed,
            max_angular=0.4,
        )

        # ─── Waypoints ───
        self._waypoints: Dict[str, Tuple[float, float, float]] = {}
        self._load_waypoints()

        # ─── State ───
        self._state_lock = threading.Lock()
        self._goal_x: Optional[float] = None
        self._goal_y: Optional[float] = None
        self._goal_theta: Optional[float] = None
        self._has_goal = False
        self._path_world: List[Tuple[float, float]] = []
        self._path_index = 0
        self._last_replan_time = 0.0
        self._nav_cycle_count = 0
        self._last_turn_dir = 0.0
        self._last_turn_time = 0.0
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0
        self._latest_scan: Optional[LD19Scan] = None
        self._latest_ranges: List[float] = []
        self._latest_angles: List[float] = []
        self._latest_intensities: List[float] = []
        self._latest_local_plan: List[Tuple[float, float]] = []

        # ─── Recovery / State Machine ───
        self._nav_state = 'IDLE'  # IDLE, PLANNING, CONTROLLING, RECOVERY
        self._recovery_behaviors = ['spin', 'backup', 'clear_costmap']
        self._recovery_index = 0
        self._recovery_start_time = 0.0
        self._stuck_start_time = time.time()
        self._last_progress_pos = (0.0, 0.0)
        self._stuck_timeout = 10.0  # seconds without progress = stuck

        # ─── SLAM / Localization ───
        global_map_size = self.get_parameter('global_map_size_m').value
        global_map_res = self.get_parameter('global_map_resolution_m').value
        self._global_map = GridMap(global_map_size, global_map_size, global_map_res, 0.15)
        self._global_map.set_origin(-global_map_size / 2.0, -global_map_size / 2.0)
        self._scan_matcher = ScanMatcher(self._global_map)
        self._ekf = EKF2D()
        self._slam_initialized = False
        self._last_scan_match_time = 0.0
        # scan_match_interval now loaded from parameter above

        # IMU yaw offset: maps absolute IMU yaw to EKF theta frame
        self._imu_yaw_offset: Optional[float] = None
        self._last_imu_yaw_for_viz = 0.0

        # Loop closure
        self._pose_graph = PoseGraph()
        self._last_keyframe_pos = (0.0, 0.0, 0.0)
        self._keyframe_dist_threshold = 0.3
        self._keyframe_angle_threshold = 0.26  # ~15 deg
        self._loop_closure_thread: Optional[threading.Thread] = None
        self._run_loop_closure = False

        # ─── MCL / Frontier / Exploration ───
        self._mcl = MCL2D()
        self._mcl.set_min_max_particles(self._mcl_particles_min, self._mcl_particles_max)
        self._mcl.set_kld_epsilon(self._mcl_kld_epsilon)
        self._mcl.set_motion_noise(self._mcl_alpha1, self._mcl_alpha2, self._mcl_alpha3, self._mcl_alpha4)
        self._mcl_active = False
        self._mcl_localization_start_time = 0.0

        self._frontier_detector = FrontierDetector()
        self._explore_engine = ExploreEngine(
            frontier_detector=self._frontier_detector,
            safety_margin_m=self._exploration_safety_margin,
            info_gain_radius_m=self._exploration_info_gain_radius,
            min_frontier_size_m=self._exploration_min_frontier_size_m,
            completion_timeout_s=self._exploration_complete_timeout,
            enable_bootstrap_spin=self._exploration_enable_bootstrap_spin,
            scoring_weights={
                "info": self._explore_weight_info,
                "distance": self._explore_weight_distance,
                "safety": self._explore_weight_safety,
                "alignment": self._explore_weight_alignment,
            },
            logger=self.get_logger(),
        )

        # ─── Start loop closure thread ───
        self._run_loop_closure = True
        self._loop_closure_thread = threading.Thread(target=self._loop_closure_worker, daemon=True)
        self._loop_closure_thread.start()

        # ─── Start hardware ───
        try:
            self._lidar.start()
            self.get_logger().info(f"LD19 lidar started on {self._lidar_port} @ {self._lidar_baud}")
        except RuntimeError as e:
            self.get_logger().error(f"Failed to start lidar: {e}")

        try:
            self._rover.connect()
            self.get_logger().info(f"Wave Rover connected on {self._rover_port}")
        except RuntimeError as e:
            self.get_logger().error(f"Failed to connect to rover: {e}")

        if self._enable_imu_yaw:
            try:
                self._imu.connect()
                self.get_logger().info(f"Yahboom IMU connected on {self._imu_port} @ {self._imu_baud}")
            except RuntimeError as e:
                self.get_logger().error(f"Failed to connect to IMU: {e}")

        # ─── Threads ───
        self._running = True
        self._lidar_consumer_thread = threading.Thread(target=self._lidar_consumer_loop, daemon=True)
        self._lidar_consumer_thread.start()

        self._nav_thread = threading.Thread(target=self._navigation_loop, daemon=True)
        self._nav_thread.start()

        # ─── Timers ───
        self._watchdog_timer = self.create_timer(0.5, self._watchdog_callback)

        self.get_logger().info(
            "AimeeNav initialized.\n"
            f"  Lidar: {self._lidar_port} @ {self._lidar_baud}\n"
            f"  Rover: {self._rover_port} @ {self._rover_baud}\n"
            f"  IMU:   {self._imu_port} @ {self._imu_baud} (yaw fusion: {self._enable_imu_yaw})\n"
            f"  Grid:  {self._grid_size}m x {self._grid_size}m @ {self._grid_res}m\n"
            f"  Mode:  {self._nav_mode}"
        )

    # ------------------------------------------------------------------
    # ROS2 callbacks
    # ------------------------------------------------------------------

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        """Receive a navigation goal."""
        with self._state_lock:
            self._goal_x = msg.pose.position.x
            self._goal_y = msg.pose.position.y
            # Extract yaw from quaternion
            q = msg.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._goal_theta = math.atan2(siny, cosy)
            self._has_goal = True
            self._path_world = []
            self._path_index = 0
            self._controller.reset()
        self._nav_state = 'GOING_TO_GOAL'
        self.get_logger().info(
            f"New goal: x={self._goal_x:.2f}, y={self._goal_y:.2f}"
        )

    def _watchdog_callback(self) -> None:
        """Periodic watchdog to check hardware health."""
        if not self._lidar.is_running():
            self.get_logger().warn("Lidar not running — attempting restart")
            try:
                self._lidar.start()
            except RuntimeError as e:
                self.get_logger().error(f"Lidar restart failed: {e}")

        if not self._rover.is_connected():
            self.get_logger().warn("Rover disconnected — attempting reconnect")
            try:
                self._rover.connect()
            except RuntimeError as e:
                self.get_logger().error(f"Rover reconnect failed: {e}")

        # Rover command watchdog
        self._rover.check_watchdog()

    # ------------------------------------------------------------------
    # Lidar consumer thread
    # ------------------------------------------------------------------

    def _lidar_consumer_loop(self) -> None:
        """Background thread: consume scans from lidar driver."""
        while self._running:
            scan = self._lidar.get_scan(block=True, timeout=0.1)
            if scan is not None:
                with self._state_lock:
                    self._latest_scan = scan
                    self._latest_ranges = [p.distance_m for p in scan.points]
                    self._latest_angles = [p.angle_deg for p in scan.points]
                    self._latest_intensities = [float(p.intensity) for p in scan.points]

    # ------------------------------------------------------------------
    # Navigation loop (main control thread)
    # ------------------------------------------------------------------

    def _navigation_loop(self) -> None:
        """Background thread: run navigation at fixed rate."""
        period = 1.0 / self._nav_rate
        overrun_warned = False
        while self._running:
            loop_start = time.time()
            try:
                self._nav_cycle()
            except Exception as e:
                import traceback
                self.get_logger().error(f"Navigation cycle error: {e}\n{traceback.format_exc()}")

            # Maintain fixed rate
            elapsed = time.time() - loop_start
            if elapsed > period and not overrun_warned:
                self.get_logger().warn(
                    f"Nav cycle overrun: {elapsed*1000:.1f}ms > {period*1000:.1f}ms "
                    f"— consider lowering nav_rate_hz or reducing grid resolution"
                )
                overrun_warned = True
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _nav_cycle(self) -> None:
        """One iteration of the navigation control loop."""
        t0 = time.time()
        prof = {}

        # ─── Get latest sensor data ───
        with self._state_lock:
            scan = self._latest_scan
            ranges = self._latest_ranges
            angles = self._latest_angles
            intensities = self._latest_intensities
            goal_x = self._goal_x
            goal_y = self._goal_y
            has_goal = self._has_goal

        if scan is None or not ranges:
            return

        # Downsample lidar for CPU efficiency on UNO Q
        if self._lidar_downsample > 1:
            ranges = ranges[::self._lidar_downsample]
            angles = angles[::self._lidar_downsample]
            intensities = intensities[::self._lidar_downsample]

        # ─── Get odometry (robot has no wheel encoders — pose is static) ───
        odom_x, odom_y, odom_theta, _, _ = self._rover.get_odometry()

        # ─── Read IMU ───
        imu_yaw_raw = self._imu.get_yaw()
        imu_gyro = self._imu.get_gyro()
        imu_yaw_scaled = None
        if imu_yaw_raw is not None:
            imu_yaw_scaled = (imu_yaw_raw * self._imu_yaw_scale +
                              math.radians(self._imu_yaw_offset_deg))
            self._last_imu_yaw_for_viz = imu_yaw_scaled

        # ─── EKF predict: real gyro for rotation, commanded linear for translation ───
        dt = 1.0 / self._nav_rate
        vth = 0.0
        if imu_gyro is not None:
            # Gyro Z in rad/s.  The Yahboom raw gyro follows the right-hand
            # rule (CCW positive) so gz is already in ROS convention.
            # NOTE: imu_yaw_scale is ONLY for yaw angle convention, not gyro.
            _, _, gz = imu_gyro
            vth = gz
        # Use previous cycle's commanded linear velocity so the EKF pose
        # advances between scan matches, giving the scan matcher a better
        # initial guess and reducing lag-induced distance under-reporting.
        self._ekf.predict(self._last_cmd_linear, vth, dt)

        # ─── EKF update: IMU yaw ───
        if self._enable_imu_yaw and imu_yaw_scaled is not None:
            if self._imu_yaw_offset is None:
                self._sync_imu_offset(imu_yaw_scaled)
            if self._imu_yaw_offset is not None:
                expected_yaw = imu_yaw_scaled + self._imu_yaw_offset
                # Normalize
                expected_yaw = math.atan2(math.sin(expected_yaw), math.cos(expected_yaw))
                self._ekf.update_imu_yaw(expected_yaw, self._imu_yaw_variance)

        # Run scan matching at reduced rate (skip if robot was stationary —
        # prevents scan-matcher drift from corrupting the idle pose)
        t_slam = time.time()
        now = time.time()
        was_stationary = (self._last_cmd_linear == 0.0 and
                          self._last_cmd_angular == 0.0)
        if (now - self._last_scan_match_time >= self._scan_match_interval and
                not was_stationary):
            self._last_scan_match_time = now
            angle_min = math.radians(angles[0]) if angles else 0.0
            angle_increment = 0.0
            if len(angles) > 1:
                angle_increment = math.radians(angles[1] - angles[0])

            if not self._slam_initialized:
                # Initialize SLAM with first scan at origin
                self._ekf.reset(0.0, 0.0, 0.0)
                self._imu_yaw_offset = None
                self._global_map.update_from_scan(
                    0.0, 0.0, 0.0,
                    ranges, angle_min, angle_increment,
                    0.01, 12.0,
                )
                self._global_map.inflate_obstacles()
                # Verify map was actually updated
                raw = self._global_map.data()
                n_free = sum(1 for v in raw if v == 0)
                n_occ = sum(1 for v in raw if v == 100)
                n_unk = sum(1 for v in raw if v == -1)
                self.get_logger().info(
                    f"SLAM initialized — map_cells=free:{n_free} occ:{n_occ} unk:{n_unk}"
                )
                self._slam_initialized = True
            else:
                init_x = self._ekf.x()
                init_y = self._ekf.y()
                init_theta = self._ekf.theta()
                match_x, match_y, match_theta, score = self._scan_matcher.match(
                    ranges, angle_min, angle_increment,
                    0.01, 12.0,
                    init_x, init_y, init_theta,
                    0.5, self._scan_match_search_angle,
                )
                if score > self._scan_match_score_threshold:
                    self._ekf.update_scan_pose(match_x, match_y, match_theta, 0.05, 0.02)
                    # NOTE: Do NOT re-sync IMU offset here. The offset should only
                    # be set once at initialization. Re-syncing on every scan match
                    # causes the IMU to validate EKF drift instead of correcting it.
                    cx = self._ekf.x()
                    cy = self._ekf.y()
                    ctheta = self._ekf.theta()

                    if self._localization_mode:
                        # Localization only: correct pose but do NOT modify the map
                        self.get_logger().info(
                            f"Localized: pose=({cx:.2f},{cy:.2f}) score={score:.1f} "
                            f"[localization mode — map unchanged]"
                        )
                    else:
                        # Mapping mode: update global map with corrected pose
                        # Idle-gate: skip map updates when robot was stationary in
                        # the previous cycle to prevent map corruption from jitter.
                        was_idle = (self._last_cmd_linear == 0.0 and
                                    self._last_cmd_angular == 0.0)
                        if not was_idle:
                            self._global_map.update_from_scan(
                                cx, cy, ctheta,
                                ranges, angle_min, angle_increment,
                                0.01, 12.0,
                            )
                            self._global_map.inflate_obstacles()
                        # Count map cells for debugging
                        raw = self._global_map.data()
                        n_free = sum(1 for v in raw if v == 0)
                        n_occ = sum(1 for v in raw if v == 100)
                        n_unk = sum(1 for v in raw if v == -1)
                        self.get_logger().info(
                            f"Scan matched: pose=({cx:.2f},{cy:.2f}) score={score:.1f} "
                            f"map_cells=free:{n_free} occ:{n_occ} unk:{n_unk}"
                        )

                        # Insert keyframe if moved far enough
                        kx, ky, kth = self._last_keyframe_pos
                        dx = cx - kx
                        dy = cy - ky
                        dth = abs(ctheta - kth)
                        while dth > math.pi:
                            dth -= 2.0 * math.pi
                        if math.hypot(dx, dy) > self._keyframe_dist_threshold or dth > self._keyframe_angle_threshold:
                            kf = Keyframe()
                            kf.x = cx
                            kf.y = cy
                            kf.theta = ctheta
                            # Store scan points in map frame
                            for i, r in enumerate(ranges):
                                if r <= 0.01 or math.isinf(r) or math.isnan(r):
                                    continue
                                a = angle_min + i * angle_increment
                                lx = r * math.cos(a)
                                ly = r * math.sin(a)
                                # Rotate to map frame
                                mx = cx + math.cos(ctheta) * lx - math.sin(ctheta) * ly
                                my = cy + math.sin(ctheta) * lx + math.cos(ctheta) * ly
                                kf.xs.append(float(mx))
                                kf.ys.append(float(my))
                            self._pose_graph.add_keyframe(kf)
                            self._last_keyframe_pos = (cx, cy, ctheta)
                            self.get_logger().info(f"Keyframe added at ({cx:.2f}, {cy:.2f})")
                else:
                    self.get_logger().info(f"Scan match FAILED: score={score:.1f}")
            prof['slam'] = time.time() - t_slam

        # Use SLAM-corrected pose for navigation
        robot_x = self._ekf.x()
        robot_y = self._ekf.y()
        robot_theta = self._ekf.theta()

        # ─── Update grid map ───
        t_grid = time.time()
        if self._enable_planning:
            self._grid.update_from_scan(
                ranges=ranges,
                angles_deg=angles,
                robot_x=robot_x,
                robot_y=robot_y,
                robot_theta=robot_theta,
                max_range_m=8.0,
            )

        prof['grid'] = time.time() - t_grid

        # ─── Update frontiers (periodic) ───
        if self._enable_exploration and not self._localization_mode:
            self._explore_engine.update_frontiers(self._global_map)

        # ─── Reactive obstacle avoidance ───
        t_react = time.time()
        points = list(zip(angles, ranges, intensities))
        sectors = self._avoidance.analyze_sectors(points)

        # Check for emergency situations
        emergency_action = self._avoidance.check_emergency(sectors)

        # ─── Check for stuck condition ───
        current_pos = (robot_x, robot_y)
        dist_moved = math.hypot(current_pos[0] - self._last_progress_pos[0],
                                current_pos[1] - self._last_progress_pos[1])
        if dist_moved > 0.1:
            self._last_progress_pos = current_pos
            self._stuck_start_time = time.time()
            if self._nav_state == 'RECOVERY':
                self._nav_state = 'CONTROLLING'
                self._recovery_index = 0

        is_stuck = (time.time() - self._stuck_start_time) > self._stuck_timeout

        # ─── Frontier exploration ───
        if not has_goal and self._enable_exploration:
            # Bootstrap spin on first exploration
            if self._explore_engine.is_bootstrap_needed(self._slam_initialized):
                self._explore_engine.start_bootstrap()
                self._nav_state = 'EXPLORING'
            elif self._explore_engine.check_completion():
                self.get_logger().info("Exploration complete — no frontiers remaining")
                self._enable_exploration = False
                if self._auto_save_on_complete:
                    self._map_manager.save_native_map(
                        map_id="auto_explore_" + time.strftime('%Y%m%d_%H%M%S'),
                        name="Auto-explored map",
                        description="Automatically saved on exploration completion",
                        global_map=self._global_map,
                        ekf=self._ekf,
                        pose_graph=self._pose_graph,
                        waypoints=self._waypoints,
                    )
            else:
                best = self._explore_engine.select_best_goal(
                    robot_x, robot_y, robot_theta,
                    self._global_map, self._grid,
                )
                if best:
                    with self._state_lock:
                        self._goal_x = best[0]
                        self._goal_y = best[1]
                        self._goal_theta = None
                        self._has_goal = True
                        self._path_world = []
                        self._path_index = 0
                    self._nav_state = 'EXPLORING'
                    self.get_logger().info(
                        f"Exploration goal: ({best[0]:.2f}, {best[1]:.2f})"
                    )
                    has_goal = True
                else:
                    # Fallback: short forward goal if no frontiers yet
                    front = next((s for s in sectors if s.name == 'front'), None)
                    front_dist = front.min_distance if front else float('inf')
                    if front_dist > 0.5:
                        bx = robot_x + 0.3 * math.cos(robot_theta)
                        by = robot_y + 0.3 * math.sin(robot_theta)
                        with self._state_lock:
                            self._goal_x = bx
                            self._goal_y = by
                            self._goal_theta = None
                            self._has_goal = True
                            self._path_world = []
                            self._path_index = 0
                        self._nav_state = 'EXPLORING'
                        self.get_logger().info(
                            "Exploration fallback: forward goal "
                            f"({bx:.2f}, {by:.2f})"
                        )
                        has_goal = True

        # ─── State machine ───
        linear_x = 0.0
        angular_z = 0.0

        if emergency_action is not None:
            linear_x, angular_z = self._avoidance.get_emergency_velocity(
                emergency_action, max_speed=self._max_speed
            )
            self.get_logger().debug(f"Emergency: {emergency_action}")

        elif self._nav_state == 'LOCALIZING':
            # MCL global localization: rotate slowly in place
            linear_x = 0.0
            angular_z = 0.4
            # Run MCL update
            if self._mcl_active and scan is not None:
                self._mcl.predict(self._last_cmd_linear, self._last_cmd_angular, dt)
                updated = self._mcl.update(
                    ranges, angle_min, angle_increment, 0.01, 12.0
                )
                if updated and self._mcl.is_converged(0.5, 0.5):
                    mx, my, mtheta = self._mcl.get_pose()
                    self._ekf.reset(mx, my, mtheta)
                    self._mcl_active = False
                    self._nav_state = 'IDLE'
                    self.get_logger().info(
                        f"MCL converged: pose=({mx:.2f},{my:.2f},{math.degrees(mtheta):.1f}°)"
                    )
                elapsed = time.time() - self._mcl_localization_start_time
                if elapsed > 30.0 and not self._mcl.is_converged(0.5, 0.5):
                    self.get_logger().warn(
                        "MCL localization timeout — please manually drive the robot in a loop"
                    )
            else:
                # No MCL active, just spin and hope scan matcher catches up
                elapsed = time.time() - self._mcl_localization_start_time
                if elapsed > 15.0:
                    self._nav_state = 'IDLE'
                    self.get_logger().warn("Localization spin timeout — resuming normal nav")

        elif self._nav_state == 'RECOVERY':
            linear_x, angular_z = self._execute_recovery()

        elif self._nav_state == 'EXPLORING' and self._explore_engine.is_bootstrap_needed(self._slam_initialized):
            linear_x, angular_z = self._explore_engine.get_bootstrap_velocity()

        elif has_goal and self._enable_planning:
            # ─── Goal-directed navigation ───
            # Re-read goal in case exploration/bootstrap just set it
            with self._state_lock:
                goal_x = self._goal_x
                goal_y = self._goal_y
            dx = goal_x - robot_x
            dy = goal_y - robot_y
            distance_to_goal = math.hypot(dx, dy)

            if distance_to_goal < self._goal_tolerance:
                # Goal reached
                with self._state_lock:
                    self._has_goal = False
                    self._path_world = []
                self._nav_state = 'IDLE'
                self.get_logger().info("Goal reached!")
                linear_x, angular_z = 0.0, 0.0

            elif is_stuck:
                self._nav_state = 'RECOVERY'
                self._recovery_start_time = time.time()
                self._recovery_index = 0
                self.get_logger().warn("Robot stuck — entering recovery")
                linear_x, angular_z = self._execute_recovery()

            else:
                # Re-plan if needed
                now = time.time()
                should_replan = (
                    not self._path_world
                    or now - self._last_replan_time > self._replan_interval
                    or self._path_index >= len(self._path_world) - 1
                )

                if should_replan:
                    # Plan on global SLAM map
                    path = self._global_planner.plan(
                        self._global_map,
                        robot_x, robot_y,
                        goal_x, goal_y,
                    )
                    if path:
                        with self._state_lock:
                            self._path_world = path
                            self._path_index = 0
                        self._last_replan_time = now
                    else:
                        # Fallback to local planner if global fails
                        path = self._planner.plan(
                            start_world=(robot_x, robot_y),
                            goal_world=(goal_x, goal_y),
                        )
                        if path is not None:
                            path = self._planner.smooth_path(path)
                            with self._state_lock:
                                self._path_world = path
                                self._path_index = 0
                            self._last_replan_time = now
                        else:
                            self.get_logger().warn("Planning failed — clearing unreachable goal")
                            with self._state_lock:
                                self._path_world = []
                                self._has_goal = False
                                self._goal_x = None
                                self._goal_y = None

                # Follow path
                if self._path_world:
                    with self._state_lock:
                        path = self._path_world
                        idx = self._path_index

                    # Advance path index
                    while idx < len(path) - 1:
                        wx, wy = path[idx]
                        dist = math.hypot(wx - robot_x, wy - robot_y)
                        if dist < 0.15:
                            idx += 1
                        else:
                            break

                    with self._state_lock:
                        self._path_index = idx

                    # Target heading = direction to next waypoint
                    if idx < len(path):
                        wx, wy = path[idx]
                        target_heading = math.atan2(wy - robot_y, wx - robot_x)

                        # Heading alignment: drive while turning (car-like)
                        heading_error = target_heading - robot_theta
                        while heading_error > math.pi:
                            heading_error -= 2.0 * math.pi
                        while heading_error < -math.pi:
                            heading_error += 2.0 * math.pi

                        # Always allow forward motion; turn penalty in controller
                        # reduces speed when heading error is large.  Pure rotation
                        # only happens at >90° error where penalty hits zero.
                        target_speed = min(self._max_speed, distance_to_goal)

                        linear_x, angular_z = self._controller.compute(
                            target_heading=target_heading,
                            current_heading=robot_theta,
                            target_speed=target_speed,
                            current_speed=0.0,
                        )

                        # Override with VFF if obstacles are close
                        goal_angle = math.degrees(target_heading)
                        fx, fy = self._avoidance.compute_vff(
                            points=points,
                            goal_angle_deg=goal_angle,
                            goal_distance_m=distance_to_goal,
                        )
                        if any(s.is_blocked for s in sectors if s.name in ('front', 'front_left', 'front_right')):
                            vff_linear, vff_angular = self._avoidance.vff_to_velocity(
                                fx, fy, max_speed=self._max_speed
                            )
                            # Blend path following with VFF
                            linear_x = 0.5 * linear_x + 0.5 * vff_linear
                            angular_z = 0.5 * angular_z + 0.5 * vff_angular
                else:
                    # No path — use VFF directly toward goal
                    goal_angle = math.degrees(math.atan2(dy, dx))
                    fx, fy = self._avoidance.compute_vff(
                        points=points,
                        goal_angle_deg=goal_angle,
                        goal_distance_m=distance_to_goal,
                    )
                    linear_x, angular_z = self._avoidance.vff_to_velocity(
                        fx, fy, max_speed=self._max_speed
                    )

        elif self._enable_reactive and (has_goal or self._enable_exploration):
            # ─── Pure reactive mode with hysteresis ───
            # Active when a goal is set OR when enable_exploration is true.
            front = next((s for s in sectors if s.name == 'front'), None)
            front_left = next((s for s in sectors if s.name == 'front_left'), None)
            front_right = next((s for s in sectors if s.name == 'front_right'), None)

            front_dist = front.min_distance if front else float('inf')
            fl_dist = front_left.min_distance if front_left else float('inf')
            fr_dist = front_right.min_distance if front_right else float('inf')

            # Exploration mode: slower, more cautious speeds
            if self._enable_exploration and not has_goal:
                speed_scale = self._exploration_speed_scale
            else:
                speed_scale = 1.0

            safety = self._safety_distance
            panic_dist = safety * 0.5
            drive_dist = safety * 1.2

            if front_dist < panic_dist:
                # Too close — reverse and turn toward more open side
                linear_x = -self._max_speed * 0.3 * speed_scale
                angular_z = 0.5 if fl_dist > fr_dist else -0.5
                self._last_turn_dir = angular_z
                self._last_turn_time = time.time()
            elif front_dist < drive_dist:
                # Caution zone — arc backward while turning toward more open side
                linear_x = -self._max_speed * 0.15 * speed_scale
                now = time.time()
                if self._last_turn_dir == 0.0:
                    self._last_turn_dir = 0.5 if fl_dist > fr_dist else -0.5
                    self._last_turn_time = now
                angular_z = self._last_turn_dir
            else:
                # Clear road ahead
                linear_x = self._max_speed * 0.4 * speed_scale
                angular_z = 0.0
                self._last_turn_dir = 0.0

                # Exploration: add small random bias to break loops and cover new ground
                if self._enable_exploration and not has_goal:
                    if random.random() < 0.03:  # 3% chance per cycle (~0.15Hz @ 5Hz)
                        angular_z = random.choice([-0.3, 0.3])

        prof['react'] = time.time() - t_react

        # ─── Send command to rover ───
        t_pub = time.time()
        self._rover.send_velocity(linear_x, angular_z)
        self._last_cmd_linear = linear_x
        self._last_cmd_angular = angular_z

        # ─── Log commanded velocity for diagnostics ───
        gx_str = f"{goal_x:.2f}" if goal_x is not None else "None"
        gy_str = f"{goal_y:.2f}" if goal_y is not None else "None"
        dist_str = f"{distance_to_goal:.3f}" if 'distance_to_goal' in locals() else "n/a"
        self.get_logger().info(
            f"CMD: linear={linear_x:.3f} angular={angular_z:.3f} "
            f"state={self._nav_state} has_goal={has_goal} "
            f"pose=({robot_x:.2f},{robot_y:.2f}) goal=({gx_str},{gy_str}) "
            f"dist={dist_str}"
        )

        # ─── Publish lightweight topics every cycle ───
        self._publish_odom(robot_x, robot_y, robot_theta, 0.0, vth)
        self._publish_transforms(robot_x, robot_y, robot_theta)
        self._publish_cmd_vel(linear_x, angular_z)
        if self._enable_imu_yaw:
            self._publish_imu()

        # ─── Publish heavy viz topics decimated ───
        self._nav_cycle_count += 1
        if self._nav_cycle_count >= self._publish_decimation:
            self._nav_cycle_count = 0
            self._publish_scan(scan, ranges, angles, intensities)
            self._publish_map()
            self._publish_path()
            self._publish_local_plan()
        prof['pub'] = time.time() - t_pub

        total = time.time() - t0
        if self._nav_cycle_count == 1 or self._nav_cycle_count % 10 == 0:
            slam_ms = prof.get('slam', 0.0) * 1000
            grid_ms = prof.get('grid', 0.0) * 1000
            react_ms = prof.get('react', 0.0) * 1000
            pub_ms = prof.get('pub', 0.0) * 1000
            other_ms = total * 1000 - slam_ms - grid_ms - react_ms - pub_ms
            self.get_logger().info(
                f"Cycle timing: total={total*1000:.1f}ms "
                f"slam={slam_ms:.1f}ms grid={grid_ms:.1f}ms "
                f"react={react_ms:.1f}ms pub={pub_ms:.1f}ms other={other_ms:.1f}ms"
            )

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _execute_recovery(self) -> Tuple[float, float]:
        """Execute current recovery behavior. Returns (linear_x, angular_z)."""
        elapsed = time.time() - self._recovery_start_time
        behavior = self._recovery_behaviors[self._recovery_index] if self._recovery_index < len(self._recovery_behaviors) else 'wait'

        if behavior == 'spin':
            # Rotate 360 degrees at 0.5 rad/s (~12 seconds)
            if elapsed > 12.0:
                self._recovery_index += 1
                self._recovery_start_time = time.time()
                self.get_logger().info("Recovery: spin complete, next behavior")
            return 0.0, 0.5

        elif behavior == 'backup':
            # Reverse for 2 seconds
            if elapsed > 2.0:
                self._recovery_index += 1
                self._recovery_start_time = time.time()
                self.get_logger().info("Recovery: backup complete, next behavior")
            return -0.15, 0.0

        elif behavior == 'clear_costmap':
            self._global_map.clear()
            self._grid.clear()
            self._recovery_index += 1
            self._recovery_start_time = time.time()
            self.get_logger().info("Recovery: costmaps cleared, next behavior")
            return 0.0, 0.0

        elif behavior == 'wait':
            if elapsed > 3.0:
                self._nav_state = 'CONTROLLING'
                self._recovery_index = 0
                self._stuck_start_time = time.time()
                self.get_logger().info("Recovery: wait complete, resuming control")
            return 0.0, 0.0

        else:
            self._nav_state = 'CONTROLLING'
            self._recovery_index = 0
            return 0.0, 0.0

    def _publish_scan(
        self,
        scan: LD19Scan,
        ranges: List[float],
        angles: List[float],
        intensities: List[float],
    ) -> None:
        """Publish LaserScan message."""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._lidar_frame_id

        # LD19: 0-360 degrees, ~1 degree resolution
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / 360.0
        msg.range_min = 0.02
        msg.range_max = 12.0
        msg.ranges = [float('inf')] * 360
        msg.intensities = [0.0] * 360

        for angle_deg, dist, intensity in zip(angles, ranges, intensities):
            idx = int(round(angle_deg)) % 360
            if 0 <= idx < 360:
                if dist < msg.ranges[idx]:
                    msg.ranges[idx] = dist
                msg.intensities[idx] = intensity

        self._scan_pub.publish(msg)

    def _publish_map(self) -> None:
        """Publish global OccupancyGrid from SLAM."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.info.resolution = self._global_map.resolution_m()
        msg.info.width = self._global_map.width_cells()
        msg.info.height = self._global_map.height_cells()
        msg.info.origin.position.x = float(self._global_map.origin_x())
        msg.info.origin.position.y = float(self._global_map.origin_y())
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        # Convert C++ grid data (-1/0/100) to ROS OccupancyGrid format
        raw = self._global_map.data()
        out = []
        for v in raw:
            if v == -1:
                out.append(-1)
            elif v == 0:
                out.append(0)
            else:
                out.append(min(100, int(v)))
        msg.data = out
        self._map_pub.publish(msg)

        # Also publish local map for debugging
        local_msg = OccupancyGrid()
        local_msg.header.stamp = msg.header.stamp
        local_msg.header.frame_id = self._map_frame
        local_msg.info.resolution = self._grid.resolution
        local_msg.info.width = self._grid.grid_size
        local_msg.info.height = self._grid.grid_size
        half = self._grid.size_m / 2.0
        cos_t = math.cos(self._grid.origin_theta)
        sin_t = math.sin(self._grid.origin_theta)
        local_msg.info.origin.position.x = float(self._grid.origin_x - half * cos_t + half * sin_t)
        local_msg.info.origin.position.y = float(self._grid.origin_y - half * sin_t - half * cos_t)
        local_msg.info.origin.position.z = 0.0
        local_msg.info.origin.orientation = self._euler_to_quaternion(0.0, 0.0, self._grid.origin_theta)
        local_msg.data = self._grid.to_occupancy_grid_data()
        self._local_map_pub.publish(local_msg)

    def _publish_odom(
        self,
        x: float, y: float, theta: float,
        vx: float, vth: float,
    ) -> None:
        """Publish Odometry message with EKF covariance."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self._euler_to_quaternion(0.0, 0.0, theta)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = vth
        # EKF covariance [x, y, theta]
        P = self._ekf.covariance()
        msg.pose.covariance[0] = float(P[0])   # x-x
        msg.pose.covariance[1] = float(P[1])   # x-y
        msg.pose.covariance[5] = float(P[2])   # x-theta
        msg.pose.covariance[6] = float(P[3])   # y-x
        msg.pose.covariance[7] = float(P[4])   # y-y
        msg.pose.covariance[11] = float(P[5])  # y-theta
        msg.pose.covariance[30] = float(P[6])  # theta-x
        msg.pose.covariance[31] = float(P[7])  # theta-y
        msg.pose.covariance[35] = float(P[8])  # theta-theta
        self._odom_pub.publish(msg)

    def _publish_transforms(self, x: float, y: float, theta: float) -> None:
        """Publish TF transforms."""
        now = self.get_clock().now().to_msg()

        # Get dead-reckoning pose for odom->base_link
        odom_x, odom_y, odom_theta, _, _ = self._rover.get_odometry()

        # map → odom (SLAM correction)
        if self._do_publish_map_odom_tf:
            dx = x - odom_x
            dy = y - odom_y
            dtheta = theta - odom_theta
            while dtheta > math.pi:
                dtheta -= 2.0 * math.pi
            while dtheta < -math.pi:
                dtheta += 2.0 * math.pi
            t1 = TransformStamped()
            t1.header.stamp = now
            t1.header.frame_id = self._map_frame
            t1.child_frame_id = self._odom_frame
            t1.transform.translation.x = float(dx)
            t1.transform.translation.y = float(dy)
            t1.transform.translation.z = 0.0
            t1.transform.rotation = self._euler_to_quaternion(0.0, 0.0, dtheta)
            self._tf_broadcaster.sendTransform(t1)

        # odom → base_link (dead reckoning)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = self._odom_frame
        t2.child_frame_id = self._base_frame
        t2.transform.translation.x = float(odom_x)
        t2.transform.translation.y = float(odom_y)
        t2.transform.translation.z = 0.0
        t2.transform.rotation = self._euler_to_quaternion(0.0, 0.0, odom_theta)
        self._tf_broadcaster.sendTransform(t2)

        # base_link → base_laser
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = self._base_frame
        t3.child_frame_id = self._lidar_frame_id
        t3.transform.translation.z = 0.18
        t3.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t3)

    def _publish_path(self) -> None:
        """Publish current planned path."""
        with self._state_lock:
            path = self._path_world.copy()

        if not path:
            return

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        for wx, wy in path:
            pose = PoseStamped()
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._path_pub.publish(msg)

    def _publish_local_plan(self) -> None:
        """Publish best DWA trajectory for RViz."""
        if not self._latest_local_plan:
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        for wx, wy in self._latest_local_plan:
            pose = PoseStamped()
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._local_plan_pub.publish(msg)

    def _publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        """Publish commanded velocity for monitoring."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)

    def _publish_imu(self) -> None:
        """Publish sensor_msgs/Imu and MagneticField from Yahboom IMU."""
        angles = self._imu.get_angles()
        gyro = self._imu.get_gyro()
        accel = self._imu.get_accel()
        mag = self._imu.get_mag()
        quat = self._imu.get_quaternion()

        now = self.get_clock().now().to_msg()

        if angles is not None and gyro is not None and accel is not None:
            msg = Imu()
            msg.header.stamp = now
            msg.header.frame_id = self._base_frame
            if quat is not None:
                msg.orientation.w = quat[0]
                msg.orientation.x = quat[1]
                msg.orientation.y = quat[2]
                msg.orientation.z = quat[3]
            else:
                msg.orientation = self._euler_to_quaternion(
                    angles[0], angles[1], angles[2]
                )
            # Angular velocity in rad/s
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
            # Linear acceleration in m/s² (g → m/s²)
            msg.linear_acceleration.x = accel[0] * 9.80665
            msg.linear_acceleration.y = accel[1] * 9.80665
            msg.linear_acceleration.z = accel[2] * 9.80665
            self._imu_pub.publish(msg)

        if mag is not None:
            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = self._base_frame
            mag_msg.magnetic_field.x = mag[0]
            mag_msg.magnetic_field.y = mag[1]
            mag_msg.magnetic_field.z = mag[2]
            self._mag_pub.publish(mag_msg)

    def _sync_imu_offset(self, imu_yaw_scaled: float) -> None:
        """Re-sync IMU yaw offset so expected_yaw matches current EKF theta."""
        self._imu_yaw_offset = self._ekf.theta() - imu_yaw_scaled
        self._imu_yaw_offset = math.atan2(
            math.sin(self._imu_yaw_offset), math.cos(self._imu_yaw_offset)
        )

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
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

    # ------------------------------------------------------------------
    # Frontier Exploration
    # ------------------------------------------------------------------

    def _find_frontiers(self, robot_x: float, robot_y: float):
        """Detect frontiers on the global SLAM map.

        Uses the global occupancy grid which covers the full explored area.
        Returns list of (world_x, world_y, size) for each frontier cluster.
        """
        import numpy as np

        w = self._global_map.width_cells()
        h = self._global_map.height_cells()
        raw = self._global_map.data()
        grid = np.array(raw, dtype=np.int8).reshape((h, w))
        res = self._global_map.resolution_m()
        origin_x = self._global_map.origin_x()
        origin_y = self._global_map.origin_y()

        # Frontier cells: free (val == 0) adjacent to unknown (val == -1)
        frontier_mask = np.zeros((h, w), dtype=bool)
        for r in range(1, h - 1):
            for c in range(1, w - 1):
                if grid[r, c] == 0:  # Free cell
                    if (grid[r - 1, c] == -1 or grid[r + 1, c] == -1 or
                            grid[r, c - 1] == -1 or grid[r, c + 1] == -1):
                        frontier_mask[r, c] = True

        if not frontier_mask.any():
            return []

        # Cluster frontier cells using BFS
        visited = np.zeros((h, w), dtype=bool)
        clusters = []

        for r in range(h):
            for c in range(w):
                if frontier_mask[r, c] and not visited[r, c]:
                    cluster = []
                    queue = [(r, c)]
                    visited[r, c] = True
                    while queue:
                        cr, cc = queue.pop(0)
                        cluster.append((cr, cc))
                        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                            nr, nc = cr + dr, cc + dc
                            if 0 <= nr < h and 0 <= nc < w:
                                if frontier_mask[nr, nc] and not visited[nr, nc]:
                                    visited[nr, nc] = True
                                    queue.append((nr, nc))

                    if len(cluster) >= self._min_frontier_size:
                        avg_r = sum(p[0] for p in cluster) / len(cluster)
                        avg_c = sum(p[1] for p in cluster) / len(cluster)
                        # Grid coords: row=y, col=x
                        wx = origin_x + avg_c * res
                        wy = origin_y + avg_r * res
                        clusters.append((wx, wy, len(cluster)))

        return clusters

    def _select_best_frontier(self, frontiers, robot_x, robot_y, robot_theta):
        """Pick the best frontier and offset it away from walls into open space.
        Only considers frontiers in the front hemisphere (±90° of heading)
        to avoid 180° spin-in-place maneuvers."""
        best = None
        best_score = -1.0

        for wx, wy, size in frontiers:
            # Skip goals inside obstacles
            if self._grid.is_occupied(wx, wy):
                continue

            dist = math.hypot(wx - robot_x, wy - robot_y)
            # Skip frontiers that are too close (avoid jitter)
            if dist < 0.3:
                continue
            # Skip frontiers that are too far (likely unreachable / behind walls)
            if dist > 3.0:
                continue

            # Front-hemisphere constraint: only pick frontiers the robot can
            # reach by turning ≤ 90°.  Eliminates 180° spin-and-overshoot.
            angle_to_frontier = math.atan2(wy - robot_y, wx - robot_x)
            heading_error = angle_to_frontier - robot_theta
            while heading_error > math.pi:
                heading_error -= 2.0 * math.pi
            while heading_error < -math.pi:
                heading_error += 2.0 * math.pi
            if abs(heading_error) > math.pi / 2.0:
                continue

            # Check line of sight on global map — reject frontiers behind walls
            if not self._has_line_of_sight(robot_x, robot_y, wx, wy):
                continue

            # Penalize frontiers surrounded by obstacles
            safety_penalty = self._frontier_safety_factor(wx, wy)

            # Offset goal toward the robot (known free space) so the robot
            # stops in open area, not hugging the wall at the frontier edge.
            dx = robot_x - wx
            dy = robot_y - wy
            d = math.hypot(dx, dy)
            if d > 0.01:
                offset = 0.5
                gx = wx + (dx / d) * offset
                gy = wy + (dy / d) * offset
            else:
                gx, gy = wx, wy

            # Re-check occupancy at offset location
            if self._grid.is_occupied(gx, gy):
                continue

            dist = math.hypot(gx - robot_x, gy - robot_y)
            score = size / (dist + 0.1) * safety_penalty
            if score > best_score:
                best_score = score
                best = (gx, gy)

        return best

    def _has_line_of_sight(self, x0, y0, x1, y1):
        """Ray-cast on global map. Return True if no occupied cells block the line."""
        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            return True

        steps = int(dist / self._global_map.resolution_m()) + 1
        for i in range(steps + 1):
            t = i / steps
            x = x0 + t * dx
            y = y0 + t * dy
            ok, gx, gy = self._global_map.world_to_grid(x, y)
            if not ok:
                continue
            val = self._global_map.cell(gx, gy)
            if val >= 50:
                return False
        return True

    def _frontier_safety_factor(self, wx, wy):
        """Return 0.1-1.0 penalty based on obstacle proximity around frontier."""
        radius = 0.25  # Check 25cm radius
        res = self._global_map.resolution_m()
        steps = max(1, int(radius / res))
        occupied = 0
        total = 0
        for dy in range(-steps, steps + 1):
            for dx in range(-steps, steps + 1):
                x = wx + dx * res
                y = wy + dy * res
                ok, gx, gy = self._global_map.world_to_grid(x, y)
                if not ok:
                    continue
                total += 1
                if self._global_map.cell(gx, gy) >= 50:
                    occupied += 1
        if total == 0:
            return 1.0
        ratio = occupied / total
        return max(0.1, 1.0 - ratio * 3.0)

    # ------------------------------------------------------------------
    # Action Server: navigate_to_pose
    # ------------------------------------------------------------------

    def _nav_goal_callback(self, goal_request):
        self.get_logger().info(f"Received navigate_to_pose goal: "
                               f"({goal_request.pose.pose.position.x:.2f}, "
                               f"{goal_request.pose.pose.position.y:.2f})")
        return GoalResponse.ACCEPT

    def _nav_cancel_callback(self, goal_handle):
        self.get_logger().info("Goal cancel requested")
        return CancelResponse.ACCEPT

    async def _navigate_to_pose_callback(self, goal_handle):
        self.get_logger().info("Executing navigate_to_pose...")
        pose = goal_request = goal_handle.request.pose.pose
        goal_x = pose.position.x
        goal_y = pose.position.y

        # Extract yaw from quaternion
        q = pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        goal_theta = math.atan2(siny, cosy)

        with self._state_lock:
            self._goal_x = goal_x
            self._goal_y = goal_y
            self._goal_theta = goal_theta
            self._has_goal = True
            self._path_world = []
            self._path_index = 0
        self._nav_state = 'GOING_TO_GOAL'

        # Wait for goal completion or cancellation
        feedback_msg = NavigateToPose.Feedback()
        rate = self.create_rate(2.0)
        while rclpy.ok() and self._running:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                with self._state_lock:
                    self._has_goal = False
                self.get_logger().info("Goal canceled")
                return NavigateToPose.Result()

            with self._state_lock:
                has_goal = self._has_goal
                robot_x = self._ekf.x()
                robot_y = self._ekf.y()
                robot_theta = self._ekf.theta()

            if not has_goal:
                # Goal reached
                goal_handle.succeed()
                self.get_logger().info("Goal succeeded")
                return NavigateToPose.Result()

            # Publish feedback
            feedback_msg.current_pose.pose.position.x = robot_x
            feedback_msg.current_pose.pose.position.y = robot_y
            feedback_msg.current_pose.pose.orientation = self._euler_to_quaternion(0.0, 0.0, robot_theta)
            feedback_msg.distance_remaining = math.hypot(goal_x - robot_x, goal_y - robot_y)
            goal_handle.publish_feedback(feedback_msg)

            try:
                rate.sleep()
            except Exception:
                break

        # Timeout or error
        goal_handle.abort()
        with self._state_lock:
            self._has_goal = False
        self.get_logger().warn("Goal aborted")
        return NavigateToPose.Result()

    # ------------------------------------------------------------------
    # Waypoints
    # ------------------------------------------------------------------

    def _load_waypoints(self) -> None:
        """Load named waypoints from YAML file."""
        if not self._waypoints_file:
            return
        path = os.path.expanduser(self._waypoints_file)
        if not os.path.exists(path):
            self.get_logger().warn(f"Waypoints file not found: {path}")
            return
        try:
            import yaml
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            if isinstance(data, dict):
                for name, pose in data.items():
                    if isinstance(pose, dict):
                        x = float(pose.get('x', 0.0))
                        y = float(pose.get('y', 0.0))
                        yaw = float(pose.get('yaw', 0.0))
                        self._waypoints[name] = (x, y, yaw)
                    elif isinstance(pose, (list, tuple)) and len(pose) >= 2:
                        x = float(pose[0])
                        y = float(pose[1])
                        yaw = float(pose[2]) if len(pose) > 2 else 0.0
                        self._waypoints[name] = (x, y, yaw)
            self.get_logger().info(
                f"Loaded {len(self._waypoints)} waypoints: {list(self._waypoints.keys())}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")

    def _on_go_to_waypoint_name(self, msg: String) -> None:
        """Navigate to a named waypoint."""
        name = msg.data.strip()
        if name not in self._waypoints:
            self.get_logger().warn(f"Unknown waypoint: '{name}'")
            return
        x, y, yaw = self._waypoints[name]
        with self._state_lock:
            self._goal_x = x
            self._goal_y = y
            self._goal_theta = yaw
            self._has_goal = True
            self._path_world = []
            self._path_index = 0
            self._controller.reset()
        self._nav_state = 'GOING_TO_GOAL'
        self.get_logger().info(f"Navigating to waypoint '{name}': ({x:.2f}, {y:.2f})")

    # ------------------------------------------------------------------
    # Map Save / Load
    # ------------------------------------------------------------------

    def _on_save_map(self, request, response):
        """Legacy save — save current map to timestamped file in flat dir."""
        try:
            os.makedirs(self._map_save_dir, exist_ok=True)
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self._map_save_dir, f'map_{timestamp}.json')
            self._save_map_to_file(filename)
            self.get_logger().info(f"Map saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")
        return response

    def _on_load_map(self, request, response):
        """Legacy load — load most recent map from flat save dir."""
        try:
            filename = self._find_latest_map_file()
            if filename is None:
                self.get_logger().warn("No saved map found to load")
                return response
            self._load_map_from_file(filename)
            self._localization_mode = True
            self.get_logger().info(
                f"Map loaded from {filename}. Localization mode ENABLED."
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
        return response

    # ------------------------------------------------------------------
    # Map Manager Service Callbacks
    # ------------------------------------------------------------------

    def _on_map_mgr_save(self, request, response):
        """Save current map into the library."""
        map_id = request.map_id.strip()
        if not map_id:
            response.success = False
            response.message = "map_id cannot be empty"
            return response
        success = self._map_manager.save_native_map(
            map_id=map_id,
            name=request.name or map_id,
            description=request.description,
            global_map=self._global_map,
            ekf=self._ekf,
            pose_graph=self._pose_graph,
            waypoints=self._waypoints,
        )
        response.success = success
        response.message = f"Map {'saved' if success else 'save failed'}: {map_id}"
        return response

    def _on_map_mgr_load(self, request, response):
        """Load a map from the library and optionally localize."""
        map_id = request.map_id.strip()
        if not map_id:
            response.success = False
            response.message = "map_id cannot be empty"
            return response

        data = self._map_manager.load_native_map(map_id)
        if data is None:
            response.success = False
            response.message = f"Map not found or corrupt: {map_id}"
            return response

        self._load_map_data(data)
        self._localization_mode = True
        self._waypoints = self._map_manager.load_waypoints(map_id)

        if request.localize:
            self._start_mcl_global_localization()

        response.success = True
        response.message = f"Map loaded: {map_id}. Localization mode ON. Waypoints: {len(self._waypoints)}"
        self.get_logger().info(response.message)
        return response

    def _on_map_mgr_list(self, request, response):
        maps = self._map_manager.list_maps()
        response.ids = [m["id"] for m in maps]
        response.names = [m.get("name", m["id"]) for m in maps]
        response.descriptions = [m.get("description", "") for m in maps]
        response.types = [m.get("type", "unknown") for m in maps]
        return response

    def _on_map_mgr_delete(self, request, response):
        map_id = request.map_id.strip()
        if not map_id:
            response.success = False
            response.message = "map_id cannot be empty"
            return response
        success = self._map_manager.delete_map(map_id)
        response.success = success
        response.message = f"Map {'deleted' if success else 'not found'}: {map_id}"
        return response

    def _on_map_mgr_import(self, request, response):
        map_id = request.map_id.strip()
        if not map_id:
            response.success = False
            response.message = "map_id cannot be empty"
            return response
        success = self._map_manager.import_ros_map(
            map_id=map_id,
            pgm_path=request.pgm_path,
            yaml_path=request.yaml_path,
            name=request.name or map_id,
            description=request.description,
        )
        response.success = success
        response.message = f"Import {'successful' if success else 'failed'}: {map_id}"
        return response

    def _on_map_mgr_export(self, request, response):
        map_id = request.map_id.strip()
        if not map_id:
            response.success = False
            response.message = "map_id cannot be empty"
            return response
        success = self._map_manager.export_ros_map(
            map_id=map_id,
            pgm_path=request.pgm_path,
            yaml_path=request.yaml_path,
        )
        response.success = success
        response.message = f"Export {'successful' if success else 'failed'}: {map_id}"
        return response

    def _on_exploration_command(self, msg: String) -> None:
        """Handle external exploration start/stop commands."""
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self._enable_exploration = True
            self._explore_engine.reset()
            self.get_logger().info("Exploration enabled via command")
        elif cmd == 'stop':
            self._enable_exploration = False
            with self._state_lock:
                self._has_goal = False
                self._goal_x = None
                self._goal_y = None
            self._nav_state = 'IDLE'
            self._rover.send_velocity(0.0, 0.0)
            self.get_logger().info("Exploration disabled via command")

    def _on_location_name(self, msg: String) -> None:
        """Load a map by location name and trigger localization."""
        location = msg.data.strip().lower()
        if not location:
            return
        # Try exact match first, then sanitize
        map_id = location.replace(" ", "_").replace("'", "").replace('"', "")
        if not self._map_manager.has_map(map_id):
            # Try to find by name (case-insensitive)
            for m in self._map_manager.list_maps():
                if m.get("name", "").strip().lower() == location:
                    map_id = m["id"]
                    break
        if not self._map_manager.has_map(map_id):
            self.get_logger().warn(f"No map found for location: {location}")
            return
        self.get_logger().info(f"Location requested: {location} -> loading map '{map_id}'")
        data = self._map_manager.load_native_map(map_id)
        if data:
            self._load_map_data(data)
            self._localization_mode = True
            self._waypoints = self._map_manager.load_waypoints(map_id)
            self.get_logger().info(
                f"Map '{map_id}' loaded for location '{location}'. Localization ON."
            )
            # Trigger global localization spin
            self._start_mcl_global_localization()

    def _load_map_data(self, data: Dict[str, Any]) -> None:
        """Load map data dict into current SLAM state."""
        map_data = data["map"]
        ekf_data = data.get("ekf", {})
        pg_data = data.get("pose_graph", {})

        # Reconstruct global map
        self._global_map.set_data(data["grid_data"])
        self._global_map.set_origin(map_data["origin_x"], map_data["origin_y"])

        # Reconstruct EKF
        self._ekf.reset(
            ekf_data.get("x", 0.0),
            ekf_data.get("y", 0.0),
            ekf_data.get("theta", 0.0),
        )
        self._imu_yaw_offset = None

        # Reconstruct pose graph
        from aimee_nav._core import PoseGraph, Keyframe
        self._pose_graph = PoseGraph()
        for kf_json in data.get("keyframes", []):
            kf = Keyframe()
            kf.x = float(kf_json["x"])
            kf.y = float(kf_json["y"])
            kf.theta = float(kf_json["theta"])
            kf.xs = [float(v) for v in kf_json.get("xs", [])]
            kf.ys = [float(v) for v in kf_json.get("ys", [])]
            self._pose_graph.add_keyframe(kf)

        for c_json in data.get("constraints", []):
            self._pose_graph.add_constraint(
                int(c_json["from"]), int(c_json["to"]),
                float(c_json["dx"]), float(c_json["dy"]),
                float(c_json["dtheta"]),
            )

        self._slam_initialized = True
        self._last_keyframe_pos = (self._ekf.x(), self._ekf.y(), self._ekf.theta())

    def _start_mcl_global_localization(self) -> None:
        """Initialize MCL for global localization and start spin."""
        self._mcl.global_localization(self._global_map, self._mcl_particles_max)
        self._mcl_active = True
        self._nav_state = 'LOCALIZING'
        self._mcl_localization_start_time = time.time()
        self.get_logger().info(
            f"MCL global localization started: {self._mcl_particles_max} particles"
        )

    def _on_set_localization_mode(self, request, response):
        """Enable or disable localization-only mode."""
        self._localization_mode = bool(request.data)
        mode_str = "ENABLED" if self._localization_mode else "DISABLED"
        self.get_logger().info(f"Localization mode {mode_str}")
        response.success = True
        response.message = f"Localization mode {mode_str}"
        return response

    def _find_latest_map_file(self) -> Optional[str]:
        """Return the most recent .json map file in the save directory."""
        if not os.path.isdir(self._map_save_dir):
            return None
        files = [
            f for f in os.listdir(self._map_save_dir)
            if f.startswith('map_') and f.endswith('.json')
        ]
        if not files:
            return None
        files.sort(reverse=True)
        return os.path.join(self._map_save_dir, files[0])

    def _save_map_to_file(self, filename: str) -> None:
        """Serialize map state to a JSON file."""
        # Grid data
        raw_grid = self._global_map.data()
        grid_bytes = bytes(b if b >= 0 else 256 + b for b in raw_grid)
        grid_b64 = base64.b64encode(grid_bytes).decode('ascii')

        # EKF state
        P = self._ekf.covariance().tolist()

        # Pose graph keyframes
        kfs = self._pose_graph.keyframes()
        keyframes_json = []
        for kf in kfs:
            keyframes_json.append({
                'x': kf.x, 'y': kf.y, 'theta': kf.theta,
                'xs': list(kf.xs), 'ys': list(kf.ys),
            })

        # Pose graph constraints
        constraints = self._pose_graph.constraints()
        constraints_json = []
        for c in constraints:
            constraints_json.append({
                'from': getattr(c, 'from'), 'to': c.to,
                'dx': c.dx, 'dy': c.dy, 'dtheta': c.dtheta,
            })

        data = {
            'version': 1,
            'map': {
                'resolution': self._global_map.resolution_m(),
                'width': self._global_map.width_cells(),
                'height': self._global_map.height_cells(),
                'origin_x': self._global_map.origin_x(),
                'origin_y': self._global_map.origin_y(),
                'grid_data_b64': grid_b64,
            },
            'ekf': {
                'x': self._ekf.x(),
                'y': self._ekf.y(),
                'theta': self._ekf.theta(),
                'covariance': P,
            },
            'pose_graph': {
                'keyframes': keyframes_json,
                'constraints': constraints_json,
            },
        }

        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

    def _load_map_from_file(self, filename: str) -> None:
        """Deserialize map state from a JSON file."""
        with open(filename, 'r') as f:
            data = json.load(f)

        map_data = data['map']
        ekf_data = data.get('ekf', {})
        pg_data = data.get('pose_graph', {})

        # Reconstruct global map
        grid_bytes = base64.b64decode(map_data['grid_data_b64'])
        grid_list = [b if b < 128 else b - 256 for b in grid_bytes]
        self._global_map.set_data(grid_list)
        self._global_map.set_origin(map_data['origin_x'], map_data['origin_y'])

        # Reconstruct EKF
        self._ekf.reset(
            ekf_data.get('x', 0.0),
            ekf_data.get('y', 0.0),
            ekf_data.get('theta', 0.0),
        )
        self._imu_yaw_offset = None
        # Covariance is not settable via public API; skip for now

        # Reconstruct pose graph
        self._pose_graph = PoseGraph()
        for kf_json in pg_data.get('keyframes', []):
            kf = Keyframe()
            kf.x = float(kf_json['x'])
            kf.y = float(kf_json['y'])
            kf.theta = float(kf_json['theta'])
            kf.xs = [float(v) for v in kf_json.get('xs', [])]
            kf.ys = [float(v) for v in kf_json.get('ys', [])]
            self._pose_graph.add_keyframe(kf)

        for c_json in pg_data.get('constraints', []):
            self._pose_graph.add_constraint(
                int(c_json['from']), int(c_json['to']),
                float(c_json['dx']), float(c_json['dy']),
                float(c_json['dtheta']),
            )

        self._slam_initialized = True
        self._last_keyframe_pos = (self._ekf.x(), self._ekf.y(), self._ekf.theta())

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _on_clear_costmap(self, request, response):
        self.get_logger().info("Clearing costmaps")
        self._global_map.clear()
        self._grid.clear()
        return response

    def _on_reinit_localization(self, request, response):
        self.get_logger().info("Reinitializing localization")
        self._ekf.reset(0.0, 0.0, 0.0)
        self._global_map.clear()
        self._slam_initialized = False
        self._imu_yaw_offset = None
        return response

    def _loop_closure_worker(self) -> None:
        """Background thread: detect loop closures and optimize pose graph."""
        while self._run_loop_closure:
            time.sleep(5.0)
            if self._localization_mode:
                continue  # Skip loop closure in localization mode
            try:
                kfs = self._pose_graph.keyframes()
                if len(kfs) < 10:
                    continue
                # Check last keyframe against older ones
                last_idx = len(kfs) - 1
                last = kfs[last_idx]
                nearby = self._pose_graph.find_nearby(last.x, last.y, 2.0)
                for idx in nearby:
                    if idx >= last_idx - 5:
                        continue  # Too recent
                    old = kfs[idx]
                    # Attempt scan-to-keyframe match
                    dx = last.x - old.x
                    dy = last.y - old.y
                    dtheta = last.theta - old.theta
                    while dtheta > math.pi:
                        dtheta -= 2.0 * math.pi
                    while dtheta < -math.pi:
                        dtheta += 2.0 * math.pi
                    # Simple constraint: just use relative pose
                    # (A real implementation would run ICP/correlation here)
                    dist = math.hypot(dx, dy)
                    if dist < 0.3 and abs(dtheta) < 0.1:
                        self._pose_graph.add_constraint(idx, last_idx, dx, dy, dtheta)
                        self.get_logger().info(f"Loop closure: keyframe {idx} <-> {last_idx}")
                        self._pose_graph.optimize(5)
                        # Update EKF with optimized last pose
                        optimized = kfs[last_idx]
                        self._ekf.update_scan_pose(optimized.x, optimized.y, optimized.theta, 0.02, 0.01)
                        break
            except Exception as e:
                self.get_logger().warn(f"Loop closure error: {e}")

    def destroy_node(self) -> None:
        """Clean shutdown."""
        self.get_logger().info("Shutting down AimeeNav...")
        self._running = False
        self._run_loop_closure = False

        # Stop rover
        try:
            self._rover.stop()
            self._rover.disconnect()
        except Exception:
            pass

        # Stop IMU
        try:
            self._imu.disconnect()
        except Exception:
            pass

        # Stop lidar
        try:
            self._lidar.stop()
        except Exception:
            pass

        # Join threads
        if self._loop_closure_thread is not None and self._loop_closure_thread.is_alive():
            self._loop_closure_thread.join(timeout=2.0)
        if self._nav_thread is not None and self._nav_thread.is_alive():
            self._nav_thread.join(timeout=2.0)
        if self._lidar_consumer_thread is not None and self._lidar_consumer_thread.is_alive():
            self._lidar_consumer_thread.join(timeout=1.0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AimeeNavNode()

    # Use MultiThreadedExecutor to allow ROS callbacks to run concurrently
    # with our background threads (though our threads are independent)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
