#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths to resources
    pkg_share = FindPackageShare(package='om_x_tb3_w_pi').find('om_x_tb3_w_pi')
    # default_model_path = os.path.join(pkg_share, 'urdf', 'combined_robot_xacro.urdf.xacro')
    default_model_path = os.path.join(pkg_share, 'urdf', 'combined_gazebo.urdf.xacro')

    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'model.rviz')

    # Launch configuration variables
    model_arg = LaunchConfiguration('model')
    rviz_config_arg = LaunchConfiguration('rviz_config')

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model_arg])}], 
        output='screen'
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_arg]
    )

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot URDF file'
        ),
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=default_rviz_config_path,
            description='Absolute path to RViz config file'
        ),
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
