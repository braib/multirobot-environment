from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
import os
from pathlib import Path
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import LaunchConfigurationNotEquals


def generate_launch_description():
    # Configuration
    gui = LaunchConfiguration('gui', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    # controller_params_file = os.path.join(get_package_share_directory('om_x_tb3_w_pi'),'config','my_controllers.yaml')

    # Paths
    default_model_path = os.path.join(
        get_package_share_directory('om_x_tb3_w_pi'),
        'urdf', 'combined_robot_xacro.urdf.xacro'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('om_x_tb3_w_pi'),
        'rviz', 'combine.rviz'
    )


    om_x_tb3_w_pi_description_path = os.path.join(
        get_package_share_directory('om_x_tb3_w_pi'))



    robot_desc = Command(
        ['xacro ', default_model_path]
    )

    params = {'robot_description': robot_desc}




    gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(om_x_tb3_w_pi_description_path, 'worlds'), ':' +
            str(Path(om_x_tb3_w_pi_description_path).parent.resolve())
            ]
        )

    arguments_ = LaunchDescription([
                DeclareLaunchArgument('world', default_value='fws_robot_world',
                          description='Ignition Gazebo World'),
           ]
    )

    gazebo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_ign_gazebo'), 'launch'), '/ign_gazebo.launch.py']),
                launch_arguments=[
                    ('ign_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 4',
                                 ' -r']
                    )
                ]
             )


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
        name='joint_state_publisher',
        parameters=[params],
        output='screen',
        condition=launch.conditions.IfCondition(gui)
    )


    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.02',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'my_robot',
                   '-allow_renaming', 'false'],
    )



    ign_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "waffle/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "waffle/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "waffle_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry"
        ]
    )


    ign_image_bridge = Node(
        package="ros_ign_image",
        executable="image_bridge",
        arguments=["/waffle_camera/image_raw"]

            
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )





    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulated clock'
        ),       

        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        gazebo_resource_path,
        arguments_, 
        gazebo_launch,
        spawn_robot,
        ign_bridge,
        ign_image_bridge,


    ])
