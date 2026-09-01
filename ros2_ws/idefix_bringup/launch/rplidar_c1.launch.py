"""RPLiDAR C1 bringup: driver node plus static TF from base_link to laser.

Uses sllidar_ros2, not ros-jazzy-rplidar-ros: the apt-packaged rplidar_ros
(v2.1.0) cannot start the C1's motor (scan-start always times out, motor
never spins, confirmed on two USB ports and every scan_mode/auto_standby
combination). sllidar_ros2 ships a dedicated C1 launch config and starts
the motor correctly. This is a deviation from the original driver lock;
see the commit message and session summary for details.

Run with: ros2 launch idefix_bringup rplidar_c1.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # RPLiDAR C1 driver via sllidar_ros2. C1 uses 460800 baud (not the
        # A1's 115200) and negotiates 'Standard' scan mode at 10 Hz.
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/idefix-lidar',
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        # Static TF base_link -> laser.
        # z = 0.125 m: deck stack (2 * floor_gap + deck_thickness = 93mm) plus
        # a measured 35mm from deck_3 top to the scan plane, +/- 10mm uncertainty.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.125',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser',
            ],
        ),
    ])
