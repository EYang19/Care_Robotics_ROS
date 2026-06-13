#!/usr/bin/env python3
"""
Teleop mapping workflow for the CareBot school-room simulation.

This launch starts Gazebo, spawns CareBot, runs slam_toolbox, opens RViz,
starts keyboard teleop, and opens a map-save terminal. Drive the robot until
the map looks complete and loop closures have settled, then press Enter in the
map-save terminal to write the map files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_plugin_path = '/opt/ros/humble/lib'
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = gazebo_plugin_path + ':' + os.environ['GAZEBO_PLUGIN_PATH']
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = gazebo_plugin_path

    workspace_model_path = os.path.dirname(
        get_package_share_directory('care_description'))
    gazebo_model_path = workspace_model_path + ':/opt/ros/humble/share'
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path + ':' + os.environ['GAZEBO_MODEL_PATH']
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    pkg_care_description = get_package_share_directory('care_description')
    pkg_care_navigation = get_package_share_directory('care_navigation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    urdf_file = os.path.join(pkg_care_description, 'urdf', 'CareBot.urdf')
    world_file = os.path.join(pkg_care_navigation, 'worlds', 'simple_hallway.world')
    slam_config = os.path.join(pkg_care_navigation, 'config', 'slam_mapping_params.yaml')
    rviz_config = os.path.join(pkg_care_navigation, 'rviz', 'care_bot_nav.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    start_teleop = LaunchConfiguration('start_teleop')
    start_map_saver = LaunchConfiguration('start_map_saver')
    map_save_path = LaunchConfiguration('map_save_path')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui': 'false',
            'server': 'true',
        }.items()
    )

    gazebo_client = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['gzclient'],
                output='screen',
                condition=IfCondition(gazebo_gui)
            )
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'care_bot',
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose
                ],
                output='screen'
            )
        ]
    )

    slam_toolbox = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'slam_params_file': slam_config
                }.items()
            )
        ]
    )

    rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen',
                condition=IfCondition(use_rviz),
                additional_env={
                    'LIBGL_ALWAYS_SOFTWARE': '1',
                    'QT_OPENGL': 'software',
                    'MESA_GL_VERSION_OVERRIDE': '3.3',
                }
            )
        ]
    )

    teleop = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal',
                    '--title=CareBot Teleop',
                    '--',
                    'bash',
                    '-lc',
                    'source /opt/ros/humble/setup.bash && '
                    'source ~/care_robotics_ws/install/setup.bash && '
                    'ros2 run teleop_twist_keyboard teleop_twist_keyboard; '
                    'echo; echo "[teleop exited - press enter to close]"; read'
                ],
                output='screen',
                condition=IfCondition(start_teleop)
            )
        ]
    )

    map_saver = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal',
                    '--title=CareBot Save Map',
                    '--',
                    'bash',
                    '-lc',
                    [
                        'source /opt/ros/humble/setup.bash && '
                        'source ~/care_robotics_ws/install/setup.bash && '
                        'echo "Drive until RViz shows a complete, stable map."; '
                        'echo "Return near known areas to let slam_toolbox close loops."; '
                        'echo "Map will be saved to: ',
                        map_save_path,
                        '"; '
                        'echo; read -p "Press Enter here when ready to save the map..."; '
                        'ros2 run nav2_map_server map_saver_cli -f ',
                        map_save_path,
                        '; echo; echo "[map saver exited - press enter to close]"; read'
                    ]
                ],
                output='screen',
                condition=IfCondition(start_map_saver)
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('start_teleop', default_value='true'),
        DeclareLaunchArgument('start_map_saver', default_value='true'),
        DeclareLaunchArgument(
            'map_save_path',
            default_value=os.path.expanduser('~/care_robotics_ws/src/care_navigation/maps/school_room')
        ),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),

        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
        slam_toolbox,
        rviz,
        teleop,
        map_saver,
    ])
