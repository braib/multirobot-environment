#!/usr/bin/env python3
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
# Authors: Arshad Mehmood

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
# from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
# from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
    

def generate_launch_description():
    ld = LaunchDescription()

    # enable_drive = LaunchConfiguration("enable_drive", default="false")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    burger_ns = LaunchConfiguration("burger_ns", default="burger")
    waffle_ns = LaunchConfiguration("waffle_ns", default="waffle")

    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    # use_sim = LaunchConfiguration('use_sim')

    # declare_enable_drive = DeclareLaunchArgument(
    #     "enable_drive", default_value="false", description="Enable robot drive node"
    # )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    declare_rviz = DeclareLaunchArgument(
        'start_rviz', default_value='false', description='Whether execute rviz2'
    )
    declare_prefix = DeclareLaunchArgument(
        'prefix', default_value='""', description='Prefix of the joint and link names'
    )

    # ld.add_action(declare_enable_drive)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz) 
    ld.add_action(declare_prefix)

    multi_robot = get_package_share_directory("multi_robot")
    launch_file_dir = os.path.join(multi_robot, "launch")

    world = os.path.join(multi_robot, "worlds", "multi_empty_world.world")

    burger_urdf_file_name = "turtlebot3_burger.urdf"
    # waffle_urdf_file_name = "turtlebot3_waffle_pi.urdf"
    # waffle_urdf_file_name = "combined_robot.urdf"


    # print("urdf_file_name : {}".format(burger_urdf_file_name))
    # print("urdf_file_name : {}".format(waffle_urdf_file_name))


    burger_urdf_ = os.path.join(
        multi_robot, "urdf", burger_urdf_file_name
    )

    # waffle_urdf_ = os.path.join(
    #     multi_robot, "urdf", waffle_urdf_file_name
    # )

    with open(burger_urdf_, 'r') as file:
        burger_urdf = file.read()

    # with open(waffle_urdf_, 'r') as file:
    #     waffle_urdf = file.read()


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

    ld.add_action(gzserver_cmd)
    # delay_gazeboclient_after_gazeboserver = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=gzserver_cmd,
    #         on_exit=[gzclient_cmd],
    #     )
    # )
    # ld.add_action(delay_gazeboclient_after_gazeboserver)
    ld.add_action(gzclient_cmd)


    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    burger_state_publisher = Node(
        package="robot_state_publisher",
        namespace="burger_ns",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time, 
            "robot_description": burger_urdf
        }],
        remappings=remappings,
        # arguments=[burger_urdf],
    )

    # Spawn robots
    spawn_turtlebot3_burger = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            # "-file", os.path.join(multi_robot, "models", "turtlebot3_burger", "model_camera.sdf"),
            "-entity", "turtlebot3_",
            "-robot_namespace", "burger_ns",
            "-x", "0.0", "-y", "0.3", "-z", "0.01", "-Y", "3.14159", "-unpause",
        ],
        output="screen",
    )


    ld.add_action(burger_state_publisher)
    ld.add_action(spawn_turtlebot3_burger)  



    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(multi_robot, "launch", "base.launch.py")
        ),
        launch_arguments={
            'start_rviz': start_rviz,
            'prefix': prefix,
            'use_sim_time': use_sim_time,
        }.items(),
    )


    # base_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(multi_robot, "launch", "base.launch.py")
    #     ),
    #    
    
    #         'start_rviz': start_rviz,
    #         'prefix': prefix,
    #         'use_sim_time': use_sim_time,
    #     }.items(),
    # )


    turtlebot3_waffle_manipulator = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace="waffle_ns",
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_manipulation_system',
            "-robot_namespace", "waffle_ns",
            "-x", "0.0", "-y", "-0.3", "-z", "0.01", "-Y", "3.14159", "-unpause",
            ],
        output='screen',
    )

    ld.add_action(base_launch)
    ld.add_action(turtlebot3_waffle_manipulator)



    # delay_wafflersp_after_burger_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_turtlebot3_burger,
    #         on_exit=[base_launch],
    #     )
    # )

    # ld.add_action(delay_wafflersp_after_burger_spawner)

    # delay_wafflespawner_after_wafflersp = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=base_launch,
    #         on_exit=[turtlebot3_waffle_manipulator],
    #     )
    # )

    # delay_wafflespawner_after_wafflersp = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=base_launch,
    #         on_exit=[turtlebot3_waffle_manipulator],
    #     )
    # )


    # ld.add_action(delay_wafflespawner_after_wafflersp)
    
    return ld
