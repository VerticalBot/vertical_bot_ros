#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='palletizer_control_pkg',
            executable='main_to_point_node.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='palletizer_control_pkg',
            executable='publish_joint_states.py',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
    ])