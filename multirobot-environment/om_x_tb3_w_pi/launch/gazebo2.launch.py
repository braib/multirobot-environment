#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Paths to resources
    multi_robot = get_package_share_directory('om_x_tb3_w_pi')
    default_model_path = os.path.join(multi_robot, 'urdf', 'combined_gazebo.urdf.xacro')
    default_rviz_config_path = os.path.join(multi_robot, 'rviz', 'model.rviz')
    world = os.path.join(multi_robot, "worlds", "empty_world.world")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    model_arg = LaunchConfiguration('model')
    rviz_config_arg = LaunchConfiguration('rviz_config')

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', model_arg]),
            'use_sim_time': use_sim_time
        }], 
        output='screen'
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
    )

    # Spawn Robot in Gazebo
    spawn_turtlebot3_burger = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            # "-topic", "robot_description",
            "-file",  Command(['xacro ', model_arg]),
            "-entity", "turtlebot3_burger",
            "-x", "0.0", "-y", "0.3", "-z", "0.01", "-Y", "3.14159", "-unpause",
        ],
        output="screen",
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
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        DeclareLaunchArgument("model", default_value=default_model_path, description="Absolute path to robot URDF file"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config_path, description="Absolute path to RViz config file"),
        joint_state_publisher_gui_node,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        spawn_turtlebot3_burger,
        rviz_node
    ])
