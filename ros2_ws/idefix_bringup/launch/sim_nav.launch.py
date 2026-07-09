# Convenience launcher: Gazebo sim + Nav2 stack in one command.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition



def generate_launch_description():
    rviz_arg = DeclareLaunchArgument('rviz',
        default_value='true',
        description='Launch RViz with the Nav2 config.'
    )
    use_rviz = LaunchConfiguration('rviz')

    rviz_config = PathJoinSubstitution([FindPackageShare('idefix_bringup'), 'rviz', 'nav2.rviz'])

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('idefix_simulation'),
            'launch', 'sim.launch.py',
        ])),
    )

    # Delay Nav2 by a few seconds so /scan, /odom, and TF are alive first.
    nav2_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('idefix_bringup'),
                'launch', 'nav2.launch.py',
            ])),
        )],
    )

    rviz_node = TimerAction(
        period=7.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=IfCondition(use_rviz), 
        )],
    )


    return LaunchDescription([
        rviz_arg,
        sim_launch,
        nav2_launch,
        rviz_node,
    ])