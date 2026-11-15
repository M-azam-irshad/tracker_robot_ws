"""Publish robot_description, spawn the robot into a running Gazebo world and start the
ROS <-> Gazebo bridge."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('tracked_robot_description')
    gz_share = get_package_share_directory('tracked_robot_gazebo')
    xacro_file = os.path.join(desc_share, 'urdf', 'tracked_robot.urdf.xacro')
    bridge_yaml = os.path.join(gz_share, 'config', 'bridge.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=true']), value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'tracked_robot',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        rsp, spawn, bridge,
    ])
