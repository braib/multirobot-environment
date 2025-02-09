#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_burger_description'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robotstatepublisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(get_package_share_directory('turtlebot3_burger_description'), 'worlds'), ':' +
            str(Path(get_package_share_directory('turtlebot3_burger_description')).parent.resolve())
        ]
    )

    arguments_ = LaunchDescription([
        DeclareLaunchArgument('world', default_value='fws_robot_world',
                             description='Ignition Gazebo World'),
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_ign_gazebo'), 'launch'), '/ign_gazebo.launch.py']),
        launch_arguments=[
            ('ign_args', [LaunchConfiguration('world'),
                          '.sdf',
                          ' -v 4',
                          ' -r']
             )
        ]
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    bridge_cmd = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image'
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(gazebo_resource_path)
    ld.add_action(arguments_)
    ld.add_action(gazebo_launch)
    ld.add_action(bridge_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld