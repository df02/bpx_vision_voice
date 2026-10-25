"""Launch the bpx_follow person-following node.

Usage:
  ros2 launch bpx_follow bpx_follow.launch.py
  ros2 launch bpx_follow bpx_follow.launch.py enabled_on_start:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('bpx_follow'), 'config', 'bpx_follow.yaml')

    config_arg = DeclareLaunchArgument(
        'follow_config', default_value=default_config,
        description='Path to bpx_follow.yaml')
    enabled_arg = DeclareLaunchArgument(
        'enabled_on_start', default_value='false',
        description='Start following immediately (default: call ~/enable)')

    bpx_follow = Node(
        package='bpx_follow',
        executable='bpx_follow_node',
        name='bpx_follow',
        parameters=[
            LaunchConfiguration('follow_config'),
            {'enabled_on_start': LaunchConfiguration('enabled_on_start')},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_arg,
        enabled_arg,
        bpx_follow,
    ])
