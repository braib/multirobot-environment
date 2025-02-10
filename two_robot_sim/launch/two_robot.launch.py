from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the URDF files
    robot1_urdf = os.path.join(get_package_share_directory('two_robot_sim'), 'urdf', 'robot1.urdf')
    robot2_urdf = os.path.join(get_package_share_directory('two_robot_sim'), 'urdf', 'robot2.urdf')

    return LaunchDescription([
        # Launch Robot 1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot1', '-file', robot1_urdf, '-x', '0.0', '-y', '0.5', '-z', '0.1'],
            output='screen'
        ),
        # Launch Robot 2
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot2', '-file', robot2_urdf, '-x', '0.0', '-y', '-0.5', '-z', '0.1'],
            output='screen'
        ),
        # Launch Gazebo with an empty world
        Node(
            package='gazebo_ros',
            executable='gazebo_ros',
            name='gazebo',
            output='screen',
            arguments=['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--verbose', 'worlds/empty.world']
        )
    ])