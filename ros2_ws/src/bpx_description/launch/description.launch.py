"""Visualize the BPX model in RViz2 without any hardware or bridge.

Publishes a static zero pose for the 12 joints so the URDF displays.
Usage:
    ros2 launch bpx_description description.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_urdf = os.path.join(
        get_package_share_directory('bpx_description'), 'urdf', 'bpx.urdf.xacro')

    urdf_arg = DeclareLaunchArgument(
        'urdf', default_value=default_urdf,
        description='Path to the BPX xacro file')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Zero-pose joint states so every link shows up in RViz.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('bpx_description'), 'config', 'description.rviz')],
        output='screen',
    )

    return LaunchDescription([
        urdf_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
