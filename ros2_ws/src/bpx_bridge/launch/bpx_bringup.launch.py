"""Bringup launch for the BPX quadruped.

Starts:
  1. robot_state_publisher with the bpx_description URDF
  2. bpx_bridge (SDK wrapper node)

Usage:
  ros2 launch bpx_bridge bpx_bringup.launch.py
  ros2 launch bpx_bridge bpx_bringup.launch.py robot_ip:=10.21.40.1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory('bpx_bridge')
    pkg_description = get_package_share_directory('bpx_description')

    default_config = os.path.join(pkg_bringup, 'config', 'bpx_bridge.yaml')
    default_urdf = os.path.join(pkg_description, 'urdf', 'bpx.urdf.xacro')

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip', default_value='10.21.20.1',
        description='BPX robot IP (RJ45 direct = 10.21.20.1, AP = 10.21.40.1)')
    config_arg = DeclareLaunchArgument(
        'bridge_config', default_value=default_config,
        description='Path to bpx_bridge.yaml')
    urdf_arg = DeclareLaunchArgument(
        'urdf', default_value=default_urdf,
        description='Path to the BPX xacro file')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }],
        output='screen',
    )

    bpx_bridge = Node(
        package='bpx_bridge',
        executable='bpx_bridge_node',
        name='bpx_bridge',
        parameters=[
            LaunchConfiguration('bridge_config'),
            {'robot_ip': LaunchConfiguration('robot_ip')},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_ip_arg,
        config_arg,
        urdf_arg,
        robot_state_publisher,
        bpx_bridge,
    ])
