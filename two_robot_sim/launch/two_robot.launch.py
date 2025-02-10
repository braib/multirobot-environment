from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package paths
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    two_robot_sim_pkg = get_package_share_directory('two_robot_sim')

    # Path configurations
    world_file = os.path.join(two_robot_sim_pkg, 'worlds', 'empty.world')
    robot1_urdf = os.path.join(two_robot_sim_pkg, 'urdf', 'robot1.urdf')
    robot2_urdf = os.path.join(two_robot_sim_pkg, 'urdf', 'robot2.urdf')

    return LaunchDescription([
        # Launch Gazebo with empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # Robot 1
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='robot1',
            parameters=[{
                'robot_description': open(robot1_urdf).read(),
                'use_sim_time': True,
                'frame_prefix': 'robot1/'  # Important for TF tree
            }],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot1',
                '-file', robot1_urdf,
                '-x', '0.0', '-y', '0.5', '-z', '0.1',
                '-robot_namespace', 'robot1',  # Namespace for ROS communications
                '-topic', 'robot1/robot_description'  # Namespaced topic
            ],
            output='screen'
        ),

        # Robot 2
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='robot2',
            parameters=[{
                'robot_description': open(robot2_urdf).read(),
                'use_sim_time': True,
                'frame_prefix': 'robot2/'  # Important for TF tree
            }],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot2',
                '-file', robot2_urdf,
                '-x', '0.0', '-y', '-0.5', '-z', '0.1',
                '-robot_namespace', 'robot2',
                '-topic', 'robot2/robot_description'
            ],
            output='screen'
        ),

        # Optional RViz (uncomment if needed)
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(two_robot_sim_pkg, 'rviz', 'two_robots.rviz')],
        #     output='screen'
        # )
    ])