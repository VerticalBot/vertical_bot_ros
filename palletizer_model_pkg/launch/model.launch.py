#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    urdf_file_name = 'palletizer_model.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))
    
    urdf = os.path.join(
        get_package_share_directory('palletizer_model_pkg'),
        'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    palletizer_pkg_dir = LaunchConfiguration(
        'palletizer_pkg_dir',
        default=os.path.join(get_package_share_directory('palletizer_control_pkg'), 'launch'))

    # print (robot_desc) # Printing urdf information.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            remappings=[
            ("/robot_description", "/palletizer/robot_description")],
            parameters=[rsp_params, {'use_sim_time': use_sim_time}]),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     output='screen',
        #     remappings=[
        #     ("joint_states", "joint_states_remap")],
        #     parameters=[rsp_params, {'use_sim_time': use_sim_time}]),
        Node(
            package='palletizer_model_pkg',
            executable='remap_pal_node.py',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [palletizer_pkg_dir, '/controller.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),  
    ])