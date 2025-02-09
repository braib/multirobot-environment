import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Declare launch arguments
    user_debug_arg = DeclareLaunchArgument(
        'user_debug', default_value='false',
        description='Enable user debug mode')

    # Paths
    package_name = 'go2_description'
    urdf_file = os.path.join(
        os.path.expanduser('~'), 'bhavish_ws', 'install', package_name, 'share', package_name, 'urdf', 'go2_description.urdf'
    )
    rviz_config_file = os.path.join(
        os.path.expanduser('~'), 'bhavish_ws', 'install', package_name, 'share', package_name, 'launch', 'model.rviz'
    )

    # Nodes
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_gui': True}]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': open(urdf_file).read()},
            {'publish_frequency': 1000.0}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        user_debug_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
