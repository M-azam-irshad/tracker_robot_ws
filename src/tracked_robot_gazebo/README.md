# tracked_robot_gazebo

Worlds (Gazebo Harmonic, all named `sar_world`):

| world | purpose |
|---|---|
| `empty` | flat ground - controller / odometry tuning |
| `test_environment` | 15 deg ramp, stairs, bumps, rubble field - locomotion + traversability tests |
| `search_and_rescue_world` | dark collapsed tunnel with side branch, rubble, 3 victims, 3 gas plumes (CO / CH4 / H2S) |

`scripts/generate_worlds.py` regenerates the `.sdf` files (seeded, reproducible).
`scripts/gas_simulator.py` simulates the gas sensors and the safe / caution / hazardous classification.
`config/bridge.yaml` lists every Gazebo <-> ROS topic.

```bash
ros2 launch tracked_robot_gazebo gazebo.launch.py world:=test_environment      # world only
ros2 launch tracked_robot_gazebo spawn_robot.launch.py                         # robot into a running world
```
(Normally just use `tracked_robot_bringup/simulation_bringup.launch.py`.)
