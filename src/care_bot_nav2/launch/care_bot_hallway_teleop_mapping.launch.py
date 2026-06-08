#!/usr/bin/env python3
"""
Compatibility wrapper for the old hallway mapping launch name.

Use care_bot_teleop_mapping.launch.py for the current school-room mapping
workflow.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_care_bot_nav2 = get_package_share_directory('care_bot_nav2')
    mapping_launch = os.path.join(
        pkg_care_bot_nav2,
        'launch',
        'care_bot_teleop_mapping.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch)
        )
    ])
