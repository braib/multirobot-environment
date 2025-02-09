#!/usr/bin/env python3

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare necessary launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = "empty.world"  # Set your desired Gazebo world file
    urdf_file_name = "turtlebot3_burger.urdf"

    # Define paths to URDF and world files
    urdf = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'urdf',
        urdf_file_name)

    world_file = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'worlds',
        world_file_name)

    # Read URDF file content
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    # Gazebo Launch File Path
    gazebo_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'launch',
        'gazebo.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='World file to load in Gazebo'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}]),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-topic', 'robot_description',
                       '-entity', 'turtlebot3_burger']),

        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gazebo_server',
            output='screen',
            arguments=['-s', 'libgazebo_ros_init.so', '-world', LaunchConfiguration('world')]),

        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gazebo_gui',
            output='screen'),
    ])
