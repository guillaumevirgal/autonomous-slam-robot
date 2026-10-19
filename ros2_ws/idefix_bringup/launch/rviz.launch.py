"""RViz2 preconfigured for slam_nav.launch.py on the real robot.

Opens rviz/slam_nav.rviz: map, both costmaps (costmap colours), global plan,
LaserScan (Best Effort), TF and RobotModel, fixed frame map, top-down view,
2D Goal Pose tool ready. No display needs to be added or adjusted by hand.

From WSL2, RViz2 runs on the Pi and displays over X11 (outbound UDP from
WSL2 never reaches the Pi's LAN, so cross-machine DDS does not work):

  ssh -O exit idefix; ssh -X idefix
  source /opt/ros/jazzy/setup.bash && source ~/robot_ws/install/setup.bash
  export ROS_DOMAIN_ID=94 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ros2 launch idefix_bringup rviz.launch.py
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('idefix_bringup'), 'rviz', 'slam_nav.rviz']
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
