#!/usr/bin/env python3
# neupan_cpp_ros launch for the 2026_sentry diff-drive sim (Level-0 test).
#
# Drives a straight-line reference from the current pose to the RViz "2D Nav
# Goal" (/goal_pose remapped to /neupan_goal), avoiding obstacles from /scan.
# Run the sentry sim (TF map->base_footprint, /scan from pointcloud_to_laserscan)
# first, then this launch, then point goals in RViz.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('neupan_cpp_ros')

    default_config = os.path.join(pkg_share, 'config', 'sentry_diff.yaml')
    default_model = os.path.join(pkg_share, 'models', 'diff_sentry.bin')

    config_file = LaunchConfiguration('config_file')
    dune_checkpoint = LaunchConfiguration('dune_checkpoint')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('dune_checkpoint', default_value=default_model),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),

        Node(
            package='neupan_cpp_ros',
            executable='neupan_node',
            name='neupan_node',
            output='screen',
            parameters=[{
                'config_file': config_file,
                'dune_checkpoint': dune_checkpoint,
                # frames from agv_bringup/navigation.launch.py
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'lidar_frame': 'base_link',  # scan target_frame is base_link
                # /scan filtering (matches pointcloud_to_laserscan range)
                'scan_angle_min': -3.14,
                'scan_angle_max': 3.14,
                'scan_range_min': 0.3,
                'scan_range_max': 8.0,
                'scan_downsample': 1,
                # path handling
                'refresh_initial_path': True,   # let a new goal override the old line
                'flip_angle': False,
                'include_initial_path_direction': False,
                # visualization
                'marker_size': 0.05,
                'marker_z': 0.2,
                # control loop
                'control_rate': 50.0,
            }],
            remappings=[
                ('/neupan_cmd_vel', cmd_vel_topic),
                ('/neupan_goal', '/goal_pose'),  # RViz "2D Nav Goal"
            ],
        ),
    ])
