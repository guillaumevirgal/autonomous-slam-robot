import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    pkg = get_package_share_directory('idefix_description')
    xacro_file = os.path.join(pkg, 'urdf', 'idefix.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'idefix.rviz')
    robot_description = ParameterValue(Command(['xacro ', xacro_file], on_stderr='ignore'), value_type=str)

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),

        Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', rviz_config]),    
     ])