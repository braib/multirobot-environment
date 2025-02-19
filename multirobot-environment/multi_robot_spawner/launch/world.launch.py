from ament_index_python import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node
import os


def generate_launch_description():


    waffle_pkg_share = launch_ros.substitutions.FindPackageShare(package='turtlebot3_manipulation_description').find('turtlebot3_manipulation_description')
    burger_pkg_share = launch_ros.substitutions.FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo')
    waffle_model_path = os.path.join(waffle_pkg_share, 'urdf/turtlebot3_manipulation.urdf.xacro')
    burger_model_path = os.path.join(burger_pkg_share, 'urdf/turtlebot3_burger.urdf')

    with open(burger_model_path, 'r') as infp:
        burger_robot_desc = infp.read()

    default_rviz_config_path = os.path.join(waffle_pkg_share, 'rviz/config.rviz')
    
    # world_path= os.path.join(get_package_share_directory('ttb_description'), 'models/worlds/house_env.world'),
    world_path=os.path.join(get_package_share_directory('bot_world'), 'worlds/home1.world')
    
    # use_sim_time = LaunchConfiguration('use_sim_time')

    waffle_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='waffle',
        parameters=[{
            'use_sim_time' : True,
            'robot_description': Command(['xacro ', waffle_model_path]),
            'frame_prefix': 'waffle/'
        }],
        output='screen'
    )
    

    burger_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='burger',
        parameters=[{
            'use_sim_time' : True,
            'robot_description': burger_robot_desc,  
            'frame_prefix': 'burger/'
        }],
        output='screen'
    )

    burger_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='burger'
    )

    waffle_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='waffle'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    gazebo_launch = launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )


    waffle_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'waffle',
            '-topic', '/waffle/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace', 'waffle'
        ],
        output='screen'
    )

    burger_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-topic', '/burger/robot_description',
            '-x', '1.0',
            '-y', '0.0',
            '-z', '0.01',
            '-robot_namespace', 'burger'
        ],
        output='screen'
    )


    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument(name='model', default_value=waffle_model_path,
                                            # description='Absolute path to robot urdf file'),
        # launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                                     description='Absolute path to rviz config file'),
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        # launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
        #                         description='Flag to enable use_sim_time'),
        gazebo_launch,
        # waffle_joint_state_publisher_node,
        burger_joint_state_publisher_node,
        # waffle_robot_state_publisher_node,
        burger_robot_state_publisher_node,
        # waffle_spawner,
        burger_spawner,
        rviz_node
    ])
