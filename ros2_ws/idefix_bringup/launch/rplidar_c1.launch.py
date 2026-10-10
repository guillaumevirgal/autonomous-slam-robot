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
        # yaw = -8.1deg (-0.1414 rad): measured 2026-09-28 by driving 0.602 m
        # straight (gyro dyaw < 0.5 deg) and ICP-matching median scans taken
        # at rest before/after: the room shifted along +8.1 deg in the laser
        # frame (|t| 0.587 m, 79% inliers < 5 cm, rms 1.4 cm), so
        # base->laser yaw = -8.1 deg. Earlier values (pi, 0, -30 deg) were
        # judged from SLAM runs while the firmware wheel base was wrong
        # (0.115 vs 0.230 m), which corrupted every rotation. ROS yaw is
        # positive counter-clockwise viewed from above.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.125',
                '--yaw', '-0.1414', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser',
            ],
        ),
    ])
