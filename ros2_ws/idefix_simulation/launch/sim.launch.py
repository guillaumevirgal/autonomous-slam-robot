##  starts Gazebo with gz sim directly
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = get_package_share_directory('idefix_description')
    sim_pkg  = get_package_share_directory('idefix_simulation')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'idefix.urdf.xacro')
    

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='idefix_lab.sdf',
        # default_value='idefix_world.sdf',
    )

    world_path = PathJoinSubstitution([
        FindPackageShare('idefix_simulation'),
        'worlds',
        LaunchConfiguration('world'),
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path], output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', '/robot_description', '-name', 'idefix', '-z', '0.05'])
    
    bridge_config = PathJoinSubstitution([
        FindPackageShare('idefix_simulation'),
        'config',
        'bridge.yaml',
    ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[
            {'config_file': bridge_config},
            {'qos_overrides./scan.publisher.reliability': 'best_effort'},
            {'qos_overrides./scan.publisher.durability': 'volatile'},
            {'qos_overrides./scan.publisher.history': 'keep_last'},
            {'qos_overrides./scan.publisher.depth': 5},
            {'qos_overrides./imu.publisher.reliability': 'best_effort'},
            {'qos_overrides./imu.publisher.durability': 'volatile'},
            {'qos_overrides./imu.publisher.history': 'keep_last'},
            {'qos_overrides./imu.publisher.depth': 10},
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        rsp,
        gazebo,
        bridge,
        TimerAction(period=4.0, actions=[spawn]),  # wait for /robot_description
    ])