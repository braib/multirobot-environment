from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
import os
from pathlib import Path
from launch.event_handlers import OnProcessExit



def generate_launch_description():


    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    gui = LaunchConfiguration('gui', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    controller_config_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'config', 'diffbot_controllers.yaml'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'rviz', 'model.rviz'
    )

    turtlebot3_burger_description_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'))


    default_model_path = os.path.join(
        get_package_share_directory('turtlebot3_burger_description'),
        'urdf', 'turtlebot3_burger_xacro.urdf.xacro'
    )

    robot_desc = Command(
        ['xacro ', default_model_path]
    )

    params = {'robot_description': robot_desc}



    gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(turtlebot3_burger_description_path, 'worlds'), ':' +
            str(Path(turtlebot3_burger_description_path).parent.resolve())
            ]
        )

    arguments_ = LaunchDescription([
                DeclareLaunchArgument('world', default_value='fws_robot_world',
                          description='Igintion Gazebo World'),
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
        parameters=[params, {'use_sim_time': True},],
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[params],
        output='screen',
        condition=UnlessCondition(gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[params],
        output='screen',
        condition=IfCondition(gui)
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
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            # "/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model",    # if this line is umcommented then it will turn off /odom and  /burger/cmd_vel topics will not be present in ignition topic list
            "/burger/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            "/burger/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/burger/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
            "/burger_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/burger/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",

        ],
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    ign_joint_states_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="ign_joint_states_bridge",
        arguments=[
            "/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model",
        ],
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    ign_image_bridge = Node(
        package="ros_ign_image",
        executable="image_bridge",
        arguments=["/burger_camera/image_raw"],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'camera.image.compressed.jpeg_quality': 75},
        ],

            
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(gui),
    )



    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[controller_config_path],
    #     output="both",
    #     remappings=[
    #         ("~/robot_description", "/robot_description"),
    #         ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
    #     ],
    # )


    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    # )

    # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # # Delay start of joint_state_broadcaster after `robot_controller`
    # # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    # delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=robot_controller_spawner,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )


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
        # delayed_spawn_robot,
        spawn_robot,
        ign_bridge,
        ign_joint_states_bridge,  
        ign_image_bridge,


        # control_node,   
        # robot_controller_spawner,
        # joint_state_broadcaster_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        # delay_joint_state_broadcaster_after_robot_controller_spawner,


    ])
