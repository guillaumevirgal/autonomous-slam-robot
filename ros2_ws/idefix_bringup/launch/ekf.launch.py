"""EKF launch for Idefix: fuses /odom and /imu/data_raw into a single
odom -> base_link estimate via robot_localization's ekf_node.

TF ownership: once this is running, ekf_filter_node is the sole publisher
of odom -> base_link (ekf.yaml sets publish_tf: true). Do NOT also run
idefix_base's odom_tf_broadcaster alongside this launch file: both would
publish the same TF edge, which is a silent-but-lethal double-broadcast
bug (looks like localization drift/oscillation later, not an obvious
error). Verify with `ros2 run tf2_tools view_frames` that odom -> base_link
has exactly one publisher, ekf_filter_node, before trusting the fused
output.

Run it with:  ros2 launch idefix_bringup ekf.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock; false on real hardware. '
                     'This launch file targets real hardware (/odom and '
                     '/imu/data_raw both come from the ESP32); sim is not '
                     'wired up to publish /imu/data_raw at all.'
    )

    bringup_share = FindPackageShare('idefix_bringup')
    ekf_params_file = PathJoinSubstitution([bringup_share, 'config', 'ekf.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
    ])
