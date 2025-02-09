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
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition



def generate_launch_description():
    gui = LaunchConfiguration('gui', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_config_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'rviz', 'model.rviz'
    )

    default_model_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'urdf', 'turtlebot3_burger_xacro.urdf.xacro'
    )

    robot_desc = Command(
        ['xacro ', default_model_path]
    )

    params = {'robot_description': robot_desc}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params, {'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(gui),
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[params],
        output='screen',
        condition=UnlessCondition(gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[params],
        output='screen',
        condition=IfCondition(gui)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
    ])