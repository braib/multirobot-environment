import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_package = get_package_share_directory('gazebo_ros')
    robot1_description = get_package_share_directory('turtlebot3_burger_description')
    robot2_description = get_package_share_directory('turtlebot3_burger_description')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_package, 'launch', 'gazebo.launch.py')
            ),
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot1',
                '-file', os.path.join(robot1_description, 'urdf', 'turtlebot3_burger.urdf'),
                '-x', '0',
                '-y', '0',
                '-z', '0.1'
                ],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot2',
                '-file', os.path.join(robot2_description, 'urdf', 'turtlebot3_burger.urdf'),
                '-x', '1',
                '-y', '0',
                '-z', '0.1'
                ],
            output='screen'
        ),
    ])
