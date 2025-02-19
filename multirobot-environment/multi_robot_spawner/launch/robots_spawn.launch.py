#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    burger_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-topic', '/burger/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace', 'burger'
        ],
        output='screen',
    )

    waffle_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'waffle',
            '-topic', '/waffle/robot_description',
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace', 'waffle'
        ],
        output='screen',
    )

    return LaunchDescription([

        burger_spawner,
        TimerAction( 
            period=3.0,
            actions=[waffle_spawner]
        )
    ])