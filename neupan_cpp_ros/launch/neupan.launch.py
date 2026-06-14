#!/usr/bin/env python3
# neupan_cpp_ros launch file.
#
# Brings up the neupan_node local planner. By default it loads the bundled
# diff-drive config and the exported DUNE model installed by libneupan, and
# tracks the global path published on /initial_path.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('neupan_cpp_ros')

    default_config = os.path.join(pkg_share, 'config', 'planner.yaml')
    default_model = os.path.join(pkg_share, 'models', 'diff_default.bin')

    config_file = LaunchConfiguration('config_file')
    dune_checkpoint = LaunchConfiguration('dune_checkpoint')
    map_frame = LaunchConfiguration('map_frame')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='Upstream-style planner.yaml'),
        DeclareLaunchArgument('dune_checkpoint', default_value=default_model,
                              description='Exported DUNE model (.bin)'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_link'),

        Node(
            package='neupan_cpp_ros',
            executable='neupan_node',
            name='neupan_node',
            output='screen',
            parameters=[{
                'config_file': config_file,
                'dune_checkpoint': dune_checkpoint,
                'map_frame': map_frame,
                'base_frame': base_frame,
                'lidar_frame': lidar_frame,
                # lidar -> obstacle point filtering
                'scan_angle_min': -3.14,
                'scan_angle_max': 3.14,
                'scan_range_min': 0.0,
                'scan_range_max': 5.0,
                'scan_downsample': 1,
                # path handling
                'refresh_initial_path': False,
                'flip_angle': False,
                'include_initial_path_direction': False,
                # visualization
                'marker_size': 0.05,
                'marker_z': 1.0,
                # control loop
                'control_rate': 50.0,
            }],
            remappings=[
                ('/scan', '/scan'),
                ('/neupan_cmd_vel', '/cmd_vel'),
            ],
        ),
    ])
