"""Autonomous frontier exploration for Idefix (explore_lite).

Run AFTER slam_nav.launch.py is up (Nav2 "Managed nodes are active"):

  ros2 launch idefix_bringup slam_nav.launch.py
  ros2 launch idefix_bringup explore.launch.py

explore_lite repeatedly sends Nav2 a goal at the most promising frontier of
slam_toolbox's /map until no frontier larger than min_frontier_size is
reachable, then returns to its start pose (config/explore.yaml).

Pause / resume:
  ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
  ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
Progress: /explore/status. Frontiers are drawn in RViz (rviz.launch.py).
Save the map afterwards with the commands in slam_mapping.launch.py.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    explore_params = PathJoinSubstitution(
        [FindPackageShare('idefix_bringup'), 'config', 'explore.yaml']
    )

    return LaunchDescription([
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[explore_params],
        ),
    ])
