"""RViz only, configured for the simulation topics (run next to an already running sim)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('tracked_robot_bringup'),
                       'config', 'simulation.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', cfg], output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
