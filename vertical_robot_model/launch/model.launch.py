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
    
    urdf_file_name = 'vertical_robot.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))
    
    urdf = os.path.join(
        get_package_share_directory('vertical_robot_model'),
        'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    # print (robot_desc) # Printing urdf information.
    rviz_config_file = os.path.join(get_package_share_directory('vertical_robot_base_pkg'),
                             'rviz', 'rviz.rviz')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}]),
        Node(
            package='vertical_robot_base_pkg',
            executable='position_control.py',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen')
            
    ])
