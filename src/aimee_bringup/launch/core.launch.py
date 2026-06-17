#!/usr/bin/env python3
"""
Core launch file for Aimee Robot
Launches essential infrastructure nodes.

All major subsystems can be toggled off for debugging:
  ros2 launch aimee_bringup core.launch.py use_voice:=false use_llm:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # ─── Launch Arguments ───
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='ron',
        description='Robot name (ron or wren)'
    )

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(
            os.getenv('AIMEE_ROBOT_WS', '/workspace'),
            'config'
        ),
        description='Path to configuration files'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Component toggles for debugging on resource-constrained targets
    use_usb_cam_arg = DeclareLaunchArgument(
        'use_usb_cam',
        default_value='false',
        description='Enable USB camera node (requires /dev/video2)'
    )

    use_voice_arg = DeclareLaunchArgument(
        'use_voice',
        default_value='true',
        description='Enable voice manager (STT) node'
    )

    use_tts_arg = DeclareLaunchArgument(
        'use_tts',
        default_value='true',
        description='Enable TTS node'
    )

    use_monitor_arg = DeclareLaunchArgument(
        'use_monitor',
        default_value='true',
        description='Enable ROS2 monitor web dashboard'
    )

    use_llm_arg = DeclareLaunchArgument(
        'use_llm',
        default_value='true',
        description='Enable local LLM backend + server'
    )

    use_intent_arg = DeclareLaunchArgument(
        'use_intent',
        default_value='true',
        description='Enable intent router'
    )

    use_skills_arg = DeclareLaunchArgument(
        'use_skills',
        default_value='true',
        description='Enable skill manager'
    )

    use_cloud_arg = DeclareLaunchArgument(
        'use_cloud',
        default_value='true',
        description='Enable AimeeCloud bridge'
    )

    audio_capture_device_arg = DeclareLaunchArgument(
        'audio_capture_device',
        default_value='default',
        description='ALSA device for audio capture'
    )

    audio_playback_device_arg = DeclareLaunchArgument(
        'audio_playback_device',
        default_value='default',
        description='ALSA device for audio playback'
    )

    cloud_ws_endpoint_arg = DeclareLaunchArgument(
        'cloud_ws_endpoint',
        default_value='wss://aimeecloud.com/ws/v1',
        description='AimeeCloud audio WebSocket endpoint'
    )

    cloud_api_key_arg = DeclareLaunchArgument(
        'cloud_api_key',
        default_value='',
        description='AimeeCloud API key'
    )

    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    config_path = LaunchConfiguration('config_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_usb_cam = LaunchConfiguration('use_usb_cam')
    use_voice = LaunchConfiguration('use_voice')
    use_tts = LaunchConfiguration('use_tts')
    use_monitor = LaunchConfiguration('use_monitor')
    use_llm = LaunchConfiguration('use_llm')
    use_intent = LaunchConfiguration('use_intent')
    use_skills = LaunchConfiguration('use_skills')
    use_cloud = LaunchConfiguration('use_cloud')
    audio_capture_device = LaunchConfiguration('audio_capture_device')
    audio_playback_device = LaunchConfiguration('audio_playback_device')
    cloud_ws_endpoint = LaunchConfiguration('cloud_ws_endpoint')
    cloud_api_key = LaunchConfiguration('cloud_api_key')

    # ─── Environment ───
    set_ros_domain_id = SetEnvironmentVariable(
        'ROS_DOMAIN_ID',
        os.getenv('ROS_DOMAIN_ID', '42')
    )
    set_pacific_tz = SetEnvironmentVariable(
        'TZ',
        'America/Los_Angeles'
    )

    # ─── Optional USB Camera Node ───
    usb_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video2',
            'image_width': 1280,
            'image_height': 720,
            'pixel_format': 'mjpeg2rgb',
            'io_method': 'mmap',
            'camera_name': 'usb_camera',
        }],
        remappings=[('image_raw', '/camera/image_raw')],
        condition=IfCondition(use_usb_cam)
    )

    # ─── Voice Pipeline ───
    voice_manager_node = Node(
        package='aimee_voice_manager',
        executable='voice_manager_node',
        name='voice_manager',
        output='screen',
        parameters=[{
            'engine': 'vosk',
            'model_path': '/home/arduino/vosk-models/vosk-model-small-en-us-0.15',
            'sample_rate': 16000,
            'audio_device': audio_capture_device,
            'publish_partials': True,
            'energy_threshold': 45.0,
            'enabled': True,
            'whisper_enabled': True,
            'whisper_api_base_url': 'https://api.lemonfox.ai/v1/audio/transcriptions',
            'whisper_api_key': os.getenv('LEMONFOX_API_KEY', ''),
            'default_voice': 'sarah',
            'stream_to_cloud': True,
        }],
        condition=IfCondition(use_voice)
    )

    tts_node = Node(
        package='aimee_tts',
        executable='tts_node',
        name='tts',
        output='screen',
        parameters=[{
            'default_engine': 'lemonfox',
            'fallback_engine': 'gtts',
            'auto_fallback': True,
            'default_voice': 'sarah',
            'lemonfox_api_key': os.getenv('LEMONFOX_API_KEY', ''),
            'lemonfox_api_base_url': 'https://api.lemonfox.ai/v1',
            'volume': 1.0,
            'audio_device': audio_playback_device,
        }],
        condition=IfCondition(use_tts)
    )

    # ─── Monitor Dashboard ───
    monitor_node = Node(
        package='aimee_ros2_monitor',
        executable='monitor_node',
        name='ros2_monitor',
        output='screen',
        condition=IfCondition(use_monitor)
    )

    # ─── Local LLM Backend (llama.cpp server) ───
    llm_backend = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'export LD_LIBRARY_PATH=/workspace/lib:$LD_LIBRARY_PATH && /workspace/lib/llama-server --host 127.0.0.1 --port 8080 -m /workspace/models/Qwen2.5-0.5B-Instruct-Q4_K_M.gguf --ctx-size 2048'
        ],
        name='llm_backend',
        output='screen',
        condition=IfCondition(use_llm)
    )

    llm_server_node = Node(
        package='aimee_llm_server',
        executable='llm_server_node',
        name='llm_server',
        output='screen',
        parameters=[{
            'backend': 'llama_cpp_server',
            'server_url': 'http://127.0.0.1:8080',
            'default_max_tokens': 150,
            'default_temperature': 0.7,
        }],
        condition=IfCondition(use_llm)
    )

    # ─── Intent & Skills ───
    intent_router_node = Node(
        package='aimee_intent_router',
        executable='intent_router_node',
        name='intent_router',
        output='screen',
        parameters=[{
            'confidence_threshold': 0.6,
            'enable_conversation_mode': True,
            'fallback_to_chat': True,
            'intent_config_path': '/workspace/config/aimee_intent_config.json',
        }],
        condition=IfCondition(use_intent)
    )

    skill_manager_node = Node(
        package='aimee_skill_manager',
        executable='skill_manager_node',
        name='skill_manager',
        output='screen',
        parameters=[{
            'default_timeout': 30.0,
            'enable_safety_checks': True,
        }],
        condition=IfCondition(use_skills)
    )

    # ─── AimeeCloud Bridge ───
    aimee_cloud_client_node = Node(
        package='aimee_cloud_bridge',
        executable='cloud_bridge_node',
        name='aimee_cloud_client',
        output='screen',
        parameters=[
            os.path.join(
                os.getenv('AIMEE_ROBOT_WS', '/workspace'),
                'src/aimee_cloud_bridge/config/cloud_bridge.yaml'
            ),
            {
                'ws_endpoint': cloud_ws_endpoint,
                'api_key': cloud_api_key,
            }
        ],
        condition=IfCondition(use_cloud)
    )

    return LaunchDescription([
        LogInfo(msg=["Starting Aimee core.launch.py"]),
        robot_name_arg,
        config_path_arg,
        use_sim_time_arg,
        use_usb_cam_arg,
        use_voice_arg,
        use_tts_arg,
        use_monitor_arg,
        use_llm_arg,
        use_intent_arg,
        use_skills_arg,
        use_cloud_arg,
        audio_capture_device_arg,
        audio_playback_device_arg,
        cloud_ws_endpoint_arg,
        cloud_api_key_arg,
        set_ros_domain_id,
        set_pacific_tz,
        usb_camera_node,
        voice_manager_node,
        tts_node,
        monitor_node,
        llm_backend,
        llm_server_node,
        intent_router_node,
        skill_manager_node,
        aimee_cloud_client_node,
    ])
