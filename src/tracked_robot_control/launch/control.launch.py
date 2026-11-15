"""Spawn ros2_control controllers (needs a running Gazebo + spawned robot) and the
command-arbitration node."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tracked_robot_control')
    diff_yaml = os.path.join(share, 'config', 'diff_drive_controller.yaml')
    limits_yaml = os.path.join(share, 'config', 'joint_limits.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen')

    diff = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager',
                   '--param-file', diff_yaml, '--controller-manager-timeout', '120'],
        output='screen')

    track_ctrl = Node(
        package='tracked_robot_control', executable='track_drive_controller',
        name='track_drive_controller', output='screen',
        parameters=[limits_yaml, {'use_sim_time': use_sim_time}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        jsb, diff, track_ctrl,
    ])
