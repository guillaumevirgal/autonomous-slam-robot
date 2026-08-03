
# nav2.launch.py
# Brings up slam_toolbox (localization mode, against a saved posegraph)
# plus the full Nav2 stack.

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    home = os.environ.get('HOME', '/root')
    default_map_yaml = os.path.join(home, 'robot_ws', 'maps', 'idefix_lab.yaml')
    default_map_serial = os.path.join(home, 'robot_ws', 'maps', 'idefix_lab')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock; false on real hardware.'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml,
        description='Full path to the .yaml map for map_server.'
    )

    map_serial_arg = DeclareLaunchArgument(
        'map_serial',
        default_value=default_map_serial,
        description='Path (no extension) to the serialized slam_toolbox '
                    'posegraph, i.e. the .data / .posegraph pair.'
    )

    bringup_share = FindPackageShare('idefix_bringup')
    nav2_params_file = PathJoinSubstitution([bringup_share, 'config', 'nav2_params.yaml'])
    slam_localization_file = PathJoinSubstitution(
        [bringup_share, 'config', 'slam_toolbox_localization.yaml']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    map_serial = LaunchConfiguration('map_serial')

    # slam_toolbox in localization mode, self-driven lifecycle
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_localization_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_serial,
            },
        ],
    )

    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # Nav2 lifecycle-managed nodes, order matters
    lifecycle_nodes = [
        'map_server',
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
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[nav2_params_file, {'yaml_filename': map_yaml}],
        ),
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
                ('cmd_vel_smoothed', 'cmd_vel'),  # output to diff-drive
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
        map_yaml_arg,
        map_serial_arg,
        slam_activate,        # register handler before emitting configure
        slam_toolbox_node,
        slam_configure,
        nav2_nodes,
    ])
