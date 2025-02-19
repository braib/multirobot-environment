from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import launch 
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to URDF file

    gui = LaunchConfiguration('gui', default='true')

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'urdf', 'turtlebot3_burger.urdf'
    )



    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    # Path to RViz config file
    rviz_config_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'rviz', 'model.rviz'
    )

    # Launch the robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params],
        output='screen'
    )

    # Launch the joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[params],
        output='screen',
        condition=launch.conditions.UnlessCondition(gui)
    )

    # Launch the joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[params],
        output='screen',
        condition=launch.conditions.IfCondition(gui)
    )

    # Include Gazebo Harmonic (ros_gz_sim) launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        )
    )

    # Spawn robot in Gazebo Harmonic using ros_gz's `create` service
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_robot', '-topic', 'robot_description'],
        output='screen'
    )

    # Launch RViz2 with configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "/scan" + "@sensor_msgs/msg/LaserScan" + "[gz.msgs.LaserScan",
        ],
        output="screen",
    )

    # Return the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        gazebo_launch,
        spawn_robot,
        rviz_node,
        gz_bridge
    ])
