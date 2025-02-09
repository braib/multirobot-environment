from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    world_file_name = LaunchConfiguration('world', default='fws_robot_world.sdf')

    # Paths
    package_path = get_package_share_directory('go2_description')
    default_model_path = os.path.join(package_path, 'urdf', 'go2_description.urdf.xacro')
    # default_model_path = '/home/bhavi/iit-gn/src/go2_description/urdf/go2_description.urdf.xacro'
    rviz_config_path = os.path.join(package_path, 'rviz', 'model.rviz')

    # Generate robot description from Xacro
    robot_desc = ParameterValue(Command(['xacro ', default_model_path]), value_type=str)


    # Set Ignition Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(package_path, 'worlds')
    )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.02',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-name', 'my_robot',
            '-allow_renaming', 'false'
        ],
        output='screen'
    )

    # Ignition Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch/ign_gazebo.launch.py')
        ),
        launch_arguments={'ign_args': [world_file_name]}.items()
    )

    # ROS-Ignition bridges
    ign_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/go2/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/go2/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist",
            "/go2_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry"
        ],
        output='screen'
    )

    ign_image_bridge = Node(
        package="ros_ign_image",
        executable="image_bridge",
        arguments=["/go2_camera/image_raw"],
        output='screen'
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Use GUI for joint state publisher'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='fws_robot_world.sdf',
            description='Ignition Gazebo world file'
        ),
        gazebo_resource_path,
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        spawn_robot,
        ign_bridge,
        ign_image_bridge,
        rviz_node
    ])
