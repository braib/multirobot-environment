import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Path to the URDF file
    urdf_file_name = 'go2_description.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('go2_description'),
        'urdf',
        urdf_file_name
    )

    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')

    # Declare launch arguments for position
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify the x-coordinate for the robot spawn location'
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify the y-coordinate for the robot spawn location'
    )

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.01',
        description='Specify the z-coordinate for the robot spawn location'
    )

    # Spawn entity node
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'go2_robot',
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    # Create launch description and add actions
    ld = LaunchDescription()

    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(spawn_entity_node)

    return ld
