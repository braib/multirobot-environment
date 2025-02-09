from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
from pathlib import Path

def generate_launch_description():
    # Configuration
    gui = LaunchConfiguration('gui', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='crazy')

    # Paths
    default_model_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'urdf', 'turtlebot3_burger.urdf'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'rviz', 'model.rviz'
    )

    

    fws_robot_sim_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'))

    fws_robot_description_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'))

    # Read robot description
    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    # Paths for custom world
    world_file_path = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_burger_description'),
        'worlds', 'crazy.sdf'  # Directly using the world file
    ])

    # Set IGN_GAZEBO_RESOURCE_PATH
    gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(fws_robot_sim_path, 'worlds'), ':' +
            str(Path(fws_robot_description_path).parent.resolve())
            ]
        )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='fws_robot_world',
                          description='Gz sim World'),
           ]
    )

    gazebo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 4',
                                 ' -r']
                    )
                ]
             )


    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[params],
        output='screen',
        condition=launch.conditions.UnlessCondition(gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[params],
        output='screen',
        condition=launch.conditions.IfCondition(gui)
    )


    # Spawn robot in Gazebo Harmonic
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_robot', '-topic', 'robot_description'],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'my_robot',
                   '-allow_renaming', 'false'],
    )


    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )

    # ROS-Gazebo bridge for LaserScan
    # gz_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     name="gz_bridge",
    #     arguments=[
    #         "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
    #     ],
    #     output="screen"
    # )




    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "/scan" + "@sensor_msgs/msg/LaserScan" + "[gz.msgs.LaserScan",
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "]gz.msgs.Twist",
        ],
        output="screen",
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        gazebo_resource_path,
        arguments,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        gazebo_launch,
        spawn_robot,
        gz_bridge,
        load_joint_state_controller,
        load_diff_drive_controller,
        rviz_node,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulated clock'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'
        )
    ])
