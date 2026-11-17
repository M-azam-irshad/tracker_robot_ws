# tracked_robot_ws - Traversability-Aware SAR Rover (NUST CEME FYP)

ROS 2 **Jazzy** + Gazebo **Harmonic** workspace containing the URDF/Xacro model of the NCRA tracked
chassis and a complete simulation: worlds, sensors (RPLIDAR, Kinect-style RGB-D, IMU), ros2_control
skid-steer drive, simulated gas sensors with safe/caution/hazardous classification, RViz and teleop.

## 1. Prerequisites (Ubuntu 24.04)
```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard python3-colcon-common-extensions
```

## 2. Build
```bash
cd tracked_robot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src -y --ignore-src   # optional
colcon build --symlink-install
source install/setup.bash
```

## 3. Run
```bash
# Look at the model only (RViz + sliders)
ros2 launch tracked_robot_description display.launch.py

# Full simulation (search & rescue tunnel)
ros2 launch tracked_robot_bringup simulation_bringup.launch.py
# other worlds / options
ros2 launch tracked_robot_bringup simulation_bringup.launch.py world:=test_environment
ros2 launch tracked_robot_bringup simulation_bringup.launch.py world:=empty rviz:=false headless:=true

# Drive (new terminal, source install/setup.bash first)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Emergency stop
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"
```
Gas: `ros2 topic echo /gas/hazard_zone`, `/gas/co_ppm`, `/gas/ch4_ppm`, `/gas/h2s_ppm`.

## 4. Packages
| package | role |
|---|---|
| tracked_robot_description | xacro model, RViz config |
| tracked_robot_gazebo | worlds, bridge, spawn, gas simulator |
| tracked_robot_control | ros2_control config + command arbitration/e-stop node |
| tracked_robot_bringup | one-command launch files |

See `docs/` for architecture, TF frames and the topic map.

## 5. IMPORTANT - what is measured vs estimated
Track/chassis geometry = your measurement table + STEP files. ESTIMATES: chassis box height (16 cm), mass split
(25 kg total given), motor torque (10 N*m per side at the sprocket, assumed). All tagged [T]/[S]/[E] in
`tracked_robot_description/urdf/robot/robot_properties.xacro`.
Tune `wheel_separation_multiplier` (diff_drive_controller.yaml) and `mu_lateral` for realistic turning.
Rocks in the worlds are <= 8 cm (this rover's roller radius is 8.45 cm).
Gas thresholds in `config/sensors.yaml` are placeholders to confirm with supervisors.
