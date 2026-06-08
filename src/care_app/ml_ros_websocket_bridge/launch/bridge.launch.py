#!/usr/bin/env python3
"""
ROS2 Launch file for Robot ROS Bridge
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description"""

    # Find package
    package_share = FindPackageShare("ml_ros_websocket_bridge")
    config_dir = PathJoinSubstitution([package_share, "config"])
    config_file = PathJoinSubstitution([config_dir, "bridge_config.yaml"])

    # Launch the bridge node
    bridge_node = Node(
        package="ml_ros_websocket_bridge",
        executable="ml_ros_websocket_bridge",
        name="ml_ros_websocket_bridge",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([bridge_node])
