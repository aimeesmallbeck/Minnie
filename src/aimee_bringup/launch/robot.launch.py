#!/usr/bin/env python3
"""
Config-driven robot bringup for Aimee Robot.

Reads a YAML robot configuration file and launches:
  • Core software stack (voice, LLM, cloud, monitor) — always consistent
  • Hardware-specific nodes (base, arm, camera) — driven by config

Usage:
  # Use default config (src/aimee_bringup/config/robots/default.yaml)
  ros2 launch aimee_bringup robot.launch.py

  # Use a specific robot config
  ROBOT_CONFIG=/workspace/config/robots/minnie.yaml \
    ros2 launch aimee_bringup robot.launch.py

  # Override software toggles for debugging
  ros2 launch aimee_bringup robot.launch.py use_voice:=false use_llm:=false
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_robot_config():
    """Load robot YAML configuration from ROBOT_CONFIG env var or default path."""
    workspace = os.getenv('AIMEE_ROBOT_WS', '/workspace')

    # Allow override via environment variable
    config_path = os.getenv('ROBOT_CONFIG')

    if not config_path:
        # Prefer AIMEE_ROBOT_NAME env var, then hostname, then default
        robot_name = os.getenv('AIMEE_ROBOT_NAME')
        if robot_name:
            named_config = os.path.join(
                workspace, 'src', 'aimee_bringup', 'config', 'robots', f'{robot_name}.yaml'
            )
            if os.path.exists(named_config):
                config_path = named_config

        if not config_path:
            # Fallback: look for a config matching hostname
            hostname = os.uname().nodename
            host_config = os.path.join(
                workspace, 'src', 'aimee_bringup', 'config', 'robots', f'{hostname}.yaml'
            )
            if os.path.exists(host_config):
                config_path = host_config
            else:
                config_path = os.path.join(
                    workspace, 'src', 'aimee_bringup', 'config', 'robots', 'default.yaml'
                )

    if not os.path.exists(config_path):
        raise FileNotFoundError(
            f"Robot config not found: {config_path}\n"
            f"Create it from the template:\n"
            f"  cp src/aimee_bringup/config/robots/default.yaml {config_path}\n"
            f"Or set ROBOT_CONFIG to point to an existing config file."
        )

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    return config, config_path


def generate_launch_description():
    # ─── Load robot config ───
    try:
        robot_cfg, config_path = load_robot_config()
    except FileNotFoundError as e:
        # If config is missing, return a launch that just logs the error
        # so the user sees it immediately.
        return LaunchDescription([
            LogInfo(msg=["ERROR: ", str(e)]),
        ])

    robot_name = robot_cfg.get('robot_name', 'aimee')
    hw = robot_cfg.get('hardware', {})
    sw = robot_cfg.get('software', {})

    # Extract hardware types early so launch args can reference them
    base_type = hw.get('base', 'none')
    arm_type = hw.get('arm', 'none')
    camera_type = hw.get('camera', 'none')
    lidar_type = hw.get('lidar', 'none')

    # ─── Robot Description (URDF) ───
    # Look for a URDF named after the robot in aimee_description
    workspace = os.getenv('AIMEE_ROBOT_WS', '/workspace')
    urdf_path = os.path.join(
        workspace, 'src', 'aimee_description', 'urdf', f'{robot_name}.urdf'
    )
    has_urdf = os.path.exists(urdf_path)
    robot_description = ''
    if has_urdf:
        with open(urdf_path, 'r') as f:
            robot_description = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ─── Launch Arguments (allow CLI overrides of software toggles) ───
    use_cloud_arg = DeclareLaunchArgument(
        'use_cloud',
        default_value=str(sw.get('cloud', True)).lower(),
        description='Enable AimeeCloud bridge'
    )
    use_voice_arg = DeclareLaunchArgument(
        'use_voice',
        default_value=str(sw.get('voice', True)).lower(),
        description='Enable voice manager (STT)'
    )
    use_tts_arg = DeclareLaunchArgument(
        'use_tts',
        default_value=str(sw.get('tts', True)).lower(),
        description='Enable TTS node'
    )
    use_monitor_arg = DeclareLaunchArgument(
        'use_monitor',
        default_value=str(sw.get('monitor', True)).lower(),
        description='Enable ROS2 monitor web dashboard'
    )
    use_llm_arg = DeclareLaunchArgument(
        'use_llm',
        default_value=str(sw.get('llm', True)).lower(),
        description='Enable local LLM backend + server'
    )
    use_intent_arg = DeclareLaunchArgument(
        'use_intent',
        default_value=str(sw.get('intent', True)).lower(),
        description='Enable intent router'
    )
    use_skills_arg = DeclareLaunchArgument(
        'use_skills',
        default_value=str(sw.get('skills', True)).lower(),
        description='Enable skill manager'
    )
    use_usb_cam_arg = DeclareLaunchArgument(
        'use_usb_cam',
        default_value=str(sw.get('usb_cam', camera_type != 'none')).lower(),
        description='Enable USB camera node (video stream)'
    )
    use_vision_arg = DeclareLaunchArgument(
        'use_vision',
        default_value=str(camera_type != 'none').lower(),
        description='Enable vision/camera pipeline'
    )
    use_arm_arg = DeclareLaunchArgument(
        'use_arm',
        default_value=str(arm_type != 'none').lower(),
        description='Enable arm/manipulation nodes'
    )
    use_base_arg = DeclareLaunchArgument(
        'use_base',
        default_value=str(base_type != 'none').lower(),
        description='Enable mobile base controller'
    )
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value=str(lidar_type != 'none').lower(),
        description='Enable lidar'
    )

    # Get launch configurations
    use_cloud = LaunchConfiguration('use_cloud')
    use_voice = LaunchConfiguration('use_voice')
    use_tts = LaunchConfiguration('use_tts')
    use_monitor = LaunchConfiguration('use_monitor')
    use_llm = LaunchConfiguration('use_llm')
    use_intent = LaunchConfiguration('use_intent')
    use_skills = LaunchConfiguration('use_skills')
    use_usb_cam = LaunchConfiguration('use_usb_cam')
    use_vision = LaunchConfiguration('use_vision')
    use_arm = LaunchConfiguration('use_arm')
    use_base = LaunchConfiguration('use_base')
    use_lidar = LaunchConfiguration('use_lidar')

    # ─── Include core launch (intelligence stack) ───
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(workspace, 'src/aimee_bringup/launch/core.launch.py')
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'use_cloud': use_cloud,
            'use_voice': use_voice,
            'use_tts': use_tts,
            'use_monitor': use_monitor,
            'use_llm': use_llm,
            'use_intent': use_intent,
            'use_skills': use_skills,
            'use_usb_cam': use_usb_cam,
        }.items()
    )

    # ─── Hardware: Mobile Base ───
    # The aimee_ugv02_controller node handles all Waveshare-protocol bases
    # (UGV02, Wave Rover, etc.).  Geometry and port are parameterized via
    # base_params so each robot gets the correct odometry without code changes.
    # Nav2 / SLAM are decoupled — they only require /odom, /cmd_vel, and
    # the odom → base_link transform, regardless of which base is underneath.
    base_params = hw.get('base_params', {})
    base_controller_node = Node(
        package='aimee_ugv02_controller',
        executable='ugv02_controller_node',
        name='base_controller',
        output='screen',
        parameters=[{
            'serial_port': base_params.get('serial_port', '/dev/ttyACM0'),
            'baud_rate': base_params.get('baud_rate', 115200),
            'wheel_separation': base_params.get('wheel_separation', 0.23),
            'wheel_radius': base_params.get('wheel_radius', 0.04),
            'max_speed': base_params.get('max_speed', 0.5),
            'publish_tf': base_params.get('publish_tf', True),
        }],
        condition=IfCondition(use_base),
    )

    # ─── Hardware: Arm ───
    arm_controller_node = Node(
        package='aimee_manipulation',
        executable='arm_controller_node',
        name='arm_controller',
        output='screen',
        parameters=[{
            'simulation_mode': False,
            'arm_type': 'roarm_m3',
        }],
        condition=IfCondition(use_arm)
    )

    pick_place_server = Node(
        package='aimee_manipulation',
        executable='pick_place_server',
        name='pick_place_server',
        output='screen',
        parameters=[{
            'default_timeout': 30.0,
        }],
        condition=IfCondition(use_arm)
    )

    # ─── Hardware: Camera / Vision Pipeline ───
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(workspace, 'src/aimee_bringup/launch/vision_pipeline.launch.py')
        ]),
        launch_arguments={
            'enable_camera': use_vision,
            'enable_manipulation': use_arm,
        }.items(),
        condition=IfCondition(use_vision)
    )

    # ─── Hardware: Lidar ───
    lidar_params = hw.get('lidar_params', {})
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'frame_id': lidar_params.get('frame_id', 'base_laser'),
            'port_name': lidar_params.get('serial_port', '/dev/ttyUSB0'),
            'port_baudrate': lidar_params.get('baud_rate', 230400),
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0,
        }],
        condition=IfCondition(use_lidar),
    )

    lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser',
        arguments=[
            '0', '0', str(lidar_params.get('height', 0.18)),
            '0', '0', '0',
            'base_link', lidar_params.get('frame_id', 'base_laser')
        ],
        condition=IfCondition(use_lidar),
    )

    # Build launch description dynamically based on hardware config
    ld = LaunchDescription([
        LogInfo(msg=[
            "Aimee robot bringup — config: ", config_path,
            " | robot: ", robot_name,
            " | base: ", base_type,
            " | arm: ", arm_type,
            " | camera: ", camera_type,
            " | lidar: ", lidar_type
        ]),
        use_cloud_arg,
        use_voice_arg,
        use_tts_arg,
        use_monitor_arg,
        use_llm_arg,
        use_intent_arg,
        use_skills_arg,
        use_vision_arg,
        use_arm_arg,
        use_base_arg,
        use_lidar_arg,
        core_launch,
    ])

    # Add base controller for Waveshare-protocol platforms.
    # To add a completely different motor platform, create a new elif block
    # here with its own controller node. Nav2 / SLAM will not need changes.
    if base_type in ('ugv02', 'wave_rover'):
        ld.add_action(base_controller_node)
    elif base_type != 'none':
        ld.add_action(LogInfo(msg=[
            "WARNING: Unknown base type '", base_type,
            "' — no base controller launched. Add it to robot.launch.py if needed."
        ]))

    # Add arm nodes if configured
    if arm_type == 'roarm_m3':
        ld.add_action(arm_controller_node)
        ld.add_action(pick_place_server)

    # Add vision pipeline if camera is configured
    if camera_type == 'obsbot':
        ld.add_action(vision_launch)

    # Add robot description publisher if URDF exists
    if has_urdf:
        ld.add_action(robot_state_publisher_node)

    # Add lidar if configured
    if lidar_type == 'ld19':
        ld.add_action(lidar_node)
        # Only publish static lidar TF if URDF doesn't already define it
        if not has_urdf:
            ld.add_action(lidar_tf_node)

    return ld
