import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Path to the URDF.XACRO file
    urdf_xacro_file_path = os.path.join(
        get_package_share_directory('go2_description'),  # Replace with your package name
        'urdf',  # Folder containing your XACRO file
        'go2_description.urdf.xacro'  # Your XACRO file
    )

    # Use xacro to process the file into a URDF
    robot_description = ParameterValue(Command(['xacro ', urdf_xacro_file_path]), value_type=str)

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher Node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # RViz2 Node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # Uncomment and modify the line below to specify an RViz configuration file
        arguments=['-d', os.path.join(get_package_share_directory('go2_description'), 'rviz', 'model.rviz')]
    )

    # Launch Description
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])
