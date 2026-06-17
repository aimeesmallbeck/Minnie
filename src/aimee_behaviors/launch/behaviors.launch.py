#!/usr/bin/env python3
"""Launch animation and marker localization behaviors."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        os.getenv("AIMEE_ROBOT_WS", "/workspace"),
        "src/aimee_behaviors/config/behaviors.yaml",
    )

    use_behaviors_arg = DeclareLaunchArgument(
        "use_behaviors",
        default_value="true",
        description="Enable behavior animation nodes",
    )
    use_marker_localization_arg = DeclareLaunchArgument(
        "use_marker_localization",
        default_value="true",
        description="Enable ArUco marker localization",
    )

    use_behaviors = LaunchConfiguration("use_behaviors")
    use_marker_localization = LaunchConfiguration("use_marker_localization")

    animation_node = Node(
        package="aimee_behaviors",
        executable="animation_node",
        name="animation_node",
        output="screen",
        parameters=[config_path],
        condition=IfCondition(use_behaviors),
    )

    marker_localization_node = Node(
        package="aimee_behaviors",
        executable="marker_localization_node",
        name="marker_localization_node",
        output="screen",
        parameters=[config_path],
        condition=IfCondition(use_marker_localization),
    )

    return LaunchDescription([
        use_behaviors_arg,
        use_marker_localization_arg,
        animation_node,
        marker_localization_node,
    ])
