import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.actions

def generate_launch_description():
    # Find the package path
    pkg_path = launch_ros.substitutions.FindPackageShare('go2_description').find('go2_description')
    default_model_path = os.path.join(pkg_path, 'urdf', 'go2_description.urdf')

    # Read the URDF file
    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    # Define nodes
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot URDF file'
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
