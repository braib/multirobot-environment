import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )

    multi_robot = get_package_share_directory("om_x_tb3_w_pi")
    launch_file_dir = os.path.join(multi_robot, "launch")

    world = os.path.join(multi_robot, "worlds", "empty_world.world")
    rviz_config_dir = os.path.join(multi_robot, "rviz", "combine.rviz")
    default_model_path = os.path.join(multi_robot, "urdf", "combined_gazebo.urdf.xacro")

    model_arg = LaunchConfiguration("model")
    rviz_config_arg = LaunchConfiguration("rviz_config")

    declare_model = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Absolute path to robot URDF file",
    )

    declare_rviz = DeclareLaunchArgument(
        name="rviz_config",
        default_value=rviz_config_dir,
        description="Absolute path to RViz config file",
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
    )

    # Correcting xacro execution
    burger_rsp = Command(["xacro ", model_arg])

    burger_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": burger_rsp,  # Corrected to use the evaluated xacro command
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_arg],
        output="screen",
    )

    # Correct way to spawn the robot
    spawn_turtlebot3_burger = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            # "-file", default_model_path,  # Pass the Xacro path, let spawn_entity.py handle it
            " -topic", "robot_description",
            "-entity", "turtlebot3_burger",
            "-x", "0.0", "-y", "0.3", "-z", "0.01", "-Y", "3.14159", "-unpause",
        ],
        output="screen",
    )

    ld.add_action(declare_model)
    ld.add_action(declare_rviz)
    ld.add_action(declare_use_sim_time)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(burger_state_publisher)
    ld.add_action(spawn_turtlebot3_burger)
    ld.add_action(rviz)

    return ld
