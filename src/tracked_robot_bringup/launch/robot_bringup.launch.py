"""HARDWARE bring-up (stub). Publishes the robot model with use_sim:=false.
TODO (Nasir): add the micro-ROS agent for the STM32 + a ros2_control hardware interface /
wheel-encoder odometry. TODO (Ahmad): add LiDAR / Kinect / IMU drivers, SLAM and Nav2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('tracked_robot_description'),
                              'urdf', 'tracked_robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false']), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
    ])
