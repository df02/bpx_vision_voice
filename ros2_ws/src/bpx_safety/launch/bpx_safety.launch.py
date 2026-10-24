"""Launch the bpx_safety depth obstacle arbiter.

Usage:
  ros2 launch bpx_safety bpx_safety.launch.py
  ros2 launch bpx_safety bpx_safety.launch.py depth_topic:=/insight9/depth
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('bpx_safety'), 'config', 'bpx_safety.yaml')

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/insight9/depth',
        description='Depth image topic (32FC1 or 16UC1)')
    config_arg = DeclareLaunchArgument(
        'safety_config', default_value=default_config,
        description='Path to bpx_safety.yaml')

    bpx_safety = Node(
        package='bpx_safety',
        executable='bpx_safety_node',
        name='bpx_safety',
        parameters=[
            LaunchConfiguration('safety_config'),
            {'depth_topic': LaunchConfiguration('depth_topic')},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        depth_topic_arg,
        config_arg,
        bpx_safety,
    ])
