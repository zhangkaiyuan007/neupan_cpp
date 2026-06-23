import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('neupan_cpp_ros')

    default_config = os.path.join(pkg_share, 'config', 'planner.yaml')
    default_model = os.path.join(pkg_share, 'models', 'diff_default.bin')

    neupan_node = Node(
        package='neupan_cpp_ros',
        executable='neupan_node',
        name='neupan_node',
        output='sreen',
        parameters=[{
            'config_file': default_config,
            'dune_checkpoint': default_model,
        }],
        remappings=[
            ('/neupan_cmd_vel', '/cmd_vel'),
        ],
    )

    astart_node = Node(
        package='neupan_cpp_ros',
        executable='astar_global_node',
        name='astar_global_node',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(neupan_node)
    ld.add_action(astart_node)

    return ld