#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('care_tof_bridge')
    default_params = os.path.join(pkg_share, 'config', 'tof_serial_bridge.yaml')

    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        Node(
            package='care_tof_bridge',
            executable='tof_serial_bridge',
            name='tof_serial_bridge',
            output='screen',
            parameters=[
                params_file,
                {
                    'serial_port': serial_port,
                    'baudrate': baudrate,
                },
            ],
        ),
    ])
