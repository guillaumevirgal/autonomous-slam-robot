"""Hardware SLAM mapping for Idefix.

Starts everything needed for a real slam_toolbox mapping session: the
RPLiDAR C1 (via rplidar_c1.launch.py), robot_state_publisher (for the
base_footprint -> base_link static link from the URDF), odom_tf_broadcaster
(odom -> base_footprint, from /odom), and slam_toolbox itself in mapping
mode.

TF tree this assembles: map (slam_toolbox) -> odom (slam_toolbox)
-> base_footprint (odom_tf_broadcaster) -> base_link (robot_state_publisher)
-> laser (rplidar_c1.launch.py's static transform).

Does not include Nav2. Run with:
  ros2 launch idefix_bringup slam_mapping.launch.py

When done mapping, save both artifact pairs:
  ros2 run nav2_map_saver map_saver_cli -f ~/robot_ws/maps/idefix_lab
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/idefix/robot_ws/maps/idefix_lab'}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    bringup_share = get_package_share_directory('idefix_bringup')
    description_share = get_package_share_directory('idefix_description')

    xacro_file = os.path.join(description_share, 'urdf', 'idefix.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file], on_stderr='ignore'), value_type=str
    )

    slam_mapping_config = os.path.join(bringup_share, 'config', 'slam_toolbox_mapping.yaml')

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'rplidar_c1.launch.py')
        )
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    odom_tf_broadcaster_node = Node(
        package='idefix_base',
        executable='odom_tf_broadcaster',
        output='screen',
    )

    # async_slam_toolbox_node is a lifecycle node: it stays unconfigured
    # (zero subscriptions, no /map) until explicitly told to configure then
    # activate. Same pattern as nav2.launch.py's slam_toolbox instance.
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_mapping_config],
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

    return LaunchDescription([
        rplidar_launch,
        robot_state_publisher_node,
        odom_tf_broadcaster_node,
        slam_activate,        # register handler before emitting configure
        slam_toolbox_node,
        slam_configure,
    ])
