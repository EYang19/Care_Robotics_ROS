#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('care_hw_bridge')
    default_params = os.path.join(pkg_share, 'config', 'stm32_serial_bridge.yaml')

    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    log_tx = LaunchConfiguration('log_tx')
    log_rx = LaunchConfiguration('log_rx')
    open_serial = LaunchConfiguration('open_serial')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('log_tx', default_value='false'),
        DeclareLaunchArgument('log_rx', default_value='false'),
        DeclareLaunchArgument('open_serial', default_value='true'),
        Node(
            package='care_hw_bridge',
            executable='stm32_serial_bridge',
            name='stm32_serial_bridge',
            output='screen',
            parameters=[
                params_file,
                {
                    'serial_port': serial_port,
                    'baudrate': baudrate,
                    'log_tx': log_tx,
                    'log_rx': log_rx,
                    'open_serial': open_serial,
                },
            ],
        ),
    ])
