"""Online SLAM + Nav2 for Idefix (navigate while mapping, no saved map).

Includes slam_mapping.launch.py unchanged (RPLiDAR C1, robot_state_publisher,
odom_tf_broadcaster, slam_toolbox in mapping mode) and adds the Nav2 stack
on top, minus map_server: the global costmap's static_layer subscribes to
slam_toolbox's live /map instead of a file, and slam_toolbox's map -> odom
TF replaces AMCL / localization-mode slam_toolbox.

Differences from nav2.launch.py (saved-map localization):
  - no map_server, no localization_slam_toolbox_node
  - no map / map_serial arguments
  - use_sim_time defaults to false (hardware-only; sim has its own launch)

Unknown space: nav2_params.yaml already sets track_unknown_space (global
costmap) and allow_unknown (NavFn), so goals can be planned through cells
slam_toolbox hasn't seen yet. Goals outside the current /map bounds are
rejected by the global costmap though; send goals inside the mapped area
or near its edge.

TF ownership: odom -> base_footprint comes from odom_tf_broadcaster (via
slam_mapping.launch.py). Do NOT run ekf.launch.py alongside this, since both
would publish into the odom -> base chain.

Run it with:
  ros2 launch idefix_bringup slam_nav.launch.py

The map can still be saved at any point with the commands documented in
slam_mapping.launch.py.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock; this launch file targets real hardware.'
    )

    nav2_delay_arg = DeclareLaunchArgument(
        'nav2_delay',
        default_value='5.0',
        description='Seconds to wait before starting Nav2, so the lidar is '
                    'spinning and slam_toolbox has published map -> odom '
                    'and a first /map.'
    )

    bringup_share = FindPackageShare('idefix_bringup')
    nav2_params_file = PathJoinSubstitution([bringup_share, 'config', 'nav2_params.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_delay = LaunchConfiguration('nav2_delay')

    slam_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, 'launch', 'slam_mapping.launch.py'])
        )
    )

    # Nav2 lifecycle-managed nodes, order matters. No map_server: /map
    # comes live from slam_toolbox.
    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    nav2_nodes = GroupAction([
        SetParameter(name='use_sim_time', value=use_sim_time),

        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen',
            parameters=[nav2_params_file],
        ),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[nav2_params_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother', executable='smoother_server',
            name='smoother_server', output='screen',
            parameters=[nav2_params_file],
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', output='screen',
            parameters=[nav2_params_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            parameters=[nav2_params_file],
        ),
        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', output='screen',
            parameters=[nav2_params_file],
        ),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen',
            parameters=[nav2_params_file],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),       # input from Nav2
                ('cmd_vel_smoothed', 'cmd_vel'),  # output to the ESP32 via micro-ROS
            ],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
                'bond_timeout': 4.0,
            }],
        ),
    ])

    return LaunchDescription([
        use_sim_time_arg,
        nav2_delay_arg,
        slam_mapping_launch,
        TimerAction(period=nav2_delay, actions=[nav2_nodes]),
    ])
