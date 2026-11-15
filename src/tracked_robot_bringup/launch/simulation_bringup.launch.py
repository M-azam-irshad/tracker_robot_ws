"""One command: Gazebo world + robot + bridge + ros2_control + gas sensors (+ RViz)."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('tracked_robot_bringup')
    gz = get_package_share_directory('tracked_robot_gazebo')
    control = get_package_share_directory('tracked_robot_control')

    with open(os.path.join(bringup, 'config', 'robot_parameters.yaml')) as fh:
        cfg = yaml.safe_load(fh)['simulation']
    sp = cfg['spawn']

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    x, y, z, yaw = (LaunchConfiguration(k) for k in ('x', 'y', 'z', 'yaw'))

    def include(pkg_share, name, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', name)),
            launch_arguments=(args or {}).items())

    gazebo = include(gz, 'gazebo.launch.py', {'world': world, 'headless': headless})
    spawn = include(gz, 'spawn_robot.launch.py', {'x': x, 'y': y, 'z': z, 'yaw': yaw})
    controllers = include(control, 'control.launch.py', {'use_sim_time': 'true'})

    gas = Node(
        package='tracked_robot_gazebo', executable='gas_simulator.py', name='gas_simulator',
        output='screen',
        parameters=[os.path.join(gz, 'config', 'sensors.yaml'),
                    {'spawn_x': x, 'spawn_y': y, 'spawn_yaw': yaw, 'use_sim_time': True}])

    # Ground-truth 'world' frame (only for RViz debugging of the plume markers).
    # The robot starts at the spawn pose, which is where SLAM's 'map' origin will be.
    world_to_map = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', x, '--y', y, '--z', '0', '--yaw', yaw,
                   '--frame-id', 'world', '--child-frame-id', 'map'],
        parameters=[{'use_sim_time': True}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(bringup, 'config', 'simulation.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(cfg['world'])),
        DeclareLaunchArgument('headless', default_value=str(cfg['headless']).lower()),
        DeclareLaunchArgument('rviz', default_value=str(cfg['rviz']).lower()),
        DeclareLaunchArgument('x', default_value=str(sp['x'])),
        DeclareLaunchArgument('y', default_value=str(sp['y'])),
        DeclareLaunchArgument('z', default_value=str(sp['z'])),
        DeclareLaunchArgument('yaw', default_value=str(sp['yaw'])),
        gazebo,
        # give gz-sim a few seconds to come up before the robot is spawned
        TimerAction(period=5.0, actions=[spawn]),
        controllers,
        gas,
        world_to_map,
        TimerAction(period=8.0, actions=[rviz]),
    ])
