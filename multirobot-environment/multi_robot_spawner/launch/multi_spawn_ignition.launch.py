import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot1_description = get_package_share_directory('turtlebot3_burger_description')
    robot2_description = get_package_share_directory('turtlebot3_burger_description')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r'],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'robot1',
                '-file', os.path.join(robot1_description, 'urdf', 'turtlebot3_burger.urdf'),
                '-x', '0',
                '-y', '0',
                '-z', '0.1'
                ],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'robot2',
                '-file', os.path.join(robot2_description, 'urdf', 'turtlebot3_burger.urdf'),
                '-x', '1',
                '-y', '0',
                '-z', '0.1'
                ],
            output='screen'
        ),
    ])
