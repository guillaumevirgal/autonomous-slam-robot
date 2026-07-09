# slam.launch.py
# Launches slam_toolbox in async online mapping mode against the sim so SLAM can be restart without reloading Gazebo

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory('idefix_simulation')

    
    slam_params = os.path.join(sim_pkg, 'config', 'slam_toolbox.yaml')

    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true', #change to False when on the Pi
        description='Use /clock sim time. true in Gazebo, false on real hardware.',
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_node,
    ])