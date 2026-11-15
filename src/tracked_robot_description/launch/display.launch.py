"""View the robot model in RViz with sliders for the wheel joints (no Gazebo)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('tracked_robot_description')
    xacro_file = os.path.join(share, 'urdf', 'tracked_robot.urdf.xacro')
    rviz_cfg = os.path.join(share, 'config', 'robot_description.rviz')
    jsp_cfg = os.path.join(share, 'config', 'joint_state_publisher.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false']), value_type=str)

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[jsp_cfg]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg], output='screen'),
    ])
