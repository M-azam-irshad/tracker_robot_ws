"""Start Gazebo Harmonic with one of the project worlds.
   world:=search_and_rescue_world | test_environment | empty   (or an absolute .sdf path)
   headless:=true runs the server only (no GUI)."""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('tracked_robot_gazebo')
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    if not os.path.isabs(world):
        world = os.path.join(share, 'worlds', world if world.endswith('.sdf') else world + '.sdf')

    flags = '-r -v 2'
    if headless:
        flags += ' -s --headless-rendering'
    gz_sim = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim),
        launch_arguments=[('gz_args', f'{flags} {world}'), ('on_exit_shutdown', 'true')])]


def generate_launch_description():
    share = get_package_share_directory('tracked_robot_gazebo')
    desc_share = get_package_share_directory('tracked_robot_description')

    # optional tread-animation plugin (package tracked_robot_gz_plugins); skipped silently if not built
    plugin_env = []
    try:
        plugin_env.append(AppendEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH', os.path.join(get_package_prefix('tracked_robot_gz_plugins'), 'lib')))
    except PackageNotFoundError:
        pass

    return LaunchDescription(plugin_env + [
        DeclareLaunchArgument('world', default_value='search_and_rescue_world'),
        DeclareLaunchArgument('headless', default_value='false'),
        # lets Gazebo resolve model:// and package:// resources (future CAD meshes)
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc_share)),
        OpaqueFunction(function=_launch_setup),
    ])
