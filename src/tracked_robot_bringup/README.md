# tracked_robot_bringup

```bash
# Full simulation: Gazebo + robot + controllers + gas sensors + RViz
ros2 launch tracked_robot_bringup simulation_bringup.launch.py
ros2 launch tracked_robot_bringup simulation_bringup.launch.py world:=test_environment
ros2 launch tracked_robot_bringup simulation_bringup.launch.py headless:=true rviz:=false x:=1.0 yaw:=0.5

# Drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Defaults come from `config/robot_parameters.yaml`.
`robot_bringup.launch.py` is the hardware entry point (STM32 / micro-ROS) - stub for now.
