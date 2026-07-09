"""Top-level bringup launch for Idefix.

A launch file starts several nodes at once with a single command, instead of
opening one terminal per node. This is the file that will eventually start the
whole robot, or the whole simulation. For now it starts the two demo nodes:
the velocity commander and the odometry reporter.

Run it with:  ros2 launch idefix_bringup bringup.launch.py
"""

from launch import LaunchDescription      # the container describing what to start
from launch_ros.actions import Node       # an action that starts one ROS2 node


def generate_launch_description():
    # Every launch file defines this function. It returns a LaunchDescription
    # holding the list of things to start.
    return LaunchDescription([

        # Start the velocity commander.
        Node(
            package='idefix_base',             # package the executable lives in
            executable='velocity_commander',   # the console_scripts name from setup.py
            name='velocity_commander',         # the node's runtime name
            output='screen',                   # send its logs to this terminal
            parameters=[{                      # override its parameters from here
                'linear_speed': 0.2,
                'angular_speed': 0.5,
            }],
        ),

        # Start the odometry reporter.
        Node(
            package='idefix_base',
            executable='odom_reporter',
            name='odom_reporter',
            output='screen',
        ),
    ])