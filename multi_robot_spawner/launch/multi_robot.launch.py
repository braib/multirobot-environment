#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    burger_pkg = get_package_share_directory('turtlebot3_gazebo')
    waffle_pkg = get_package_share_directory('turtlebot3_manipulation_description')

    # World file
    world_file = os.path.join(burger_pkg, 'worlds', 'empty_world.world')

    # Robot descriptions
    burger_urdf = os.path.join(burger_pkg, 'urdf', 'turtlebot3_burger.urdf')
    waffle_xacro = os.path.join(waffle_pkg, 'urdf', 'turtlebot3_manipulation.urdf.xacro')

    with open(burger_urdf, 'r') as infp:
        burger_robot_desc = infp.read()

    waffle_robot_desc = Command(['xacro ', waffle_xacro])

    # Launch Gazebo Classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
        }.items()
    )

    # Robot 1 - URDF-based
    burger_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='burger',
        parameters=[{
            'robot_description': burger_robot_desc,
            'frame_prefix': 'burger/'
        }],
        output='screen'
    )

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
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        gazebo,
        burger_state_publisher,
        burger_spawner,
    ])