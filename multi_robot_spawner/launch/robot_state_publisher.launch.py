#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    burger_urdf_path = os.path.join( get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_burger.urdf')

    with open(burger_urdf_path, 'r') as infp:
        burger_robot_desc = infp.read()

    waffle_urdf_path = os.path.join( get_package_share_directory('turtlebot3_manipulation_description'), 'urdf', 'turtlebot3_manipulation.urdf.xacro')

    waffle_robot_desc = Command(['xacro ', waffle_urdf_path])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='burger',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': burger_robot_desc,
                'frame_prefix': 'burger/'
            }],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='waffle',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': waffle_robot_desc,
                'frame_prefix': 'waffle/'
            }],
            output='screen'
        )
    ])