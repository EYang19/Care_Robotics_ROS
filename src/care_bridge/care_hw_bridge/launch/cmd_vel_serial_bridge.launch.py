#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('care_hw_bridge')
    default_params = os.path.join(pkg_share, 'config', 'cmd_vel_serial_bridge.yaml')

    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    protocol = LaunchConfiguration('protocol')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('protocol', default_value='ascii'),
        Node(
            package='care_hw_bridge',
            executable='cmd_vel_serial_bridge',
            name='cmd_vel_serial_bridge',
            output='screen',
            parameters=[
                params_file,
                {
                    'serial_port': serial_port,
                    'baudrate': baudrate,
                    'protocol': protocol,
                },
            ],
        ),
    ])
