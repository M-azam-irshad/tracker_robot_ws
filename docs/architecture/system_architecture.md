# System architecture (simulation)

```
Gazebo Harmonic (world sar_world)
  |- robot model (URDF via ros_gz_sim create)
  |- gz_ros2_control plugin -> controller_manager -> joint_state_broadcaster, diff_drive_controller
  |- sensors: gpu_lidar, rgbd_camera, imu
  `- ros_gz_bridge (config/bridge.yaml) <-> ROS 2

teleop / Nav2 -> track_drive_controller -> /diff_drive_controller/cmd_vel -> 8 wheel joints
gas_simulator.py (ground-truth pose + plumes) -> /gas/* topics, hazard zone
```
Planned (other team members): SLAM Toolbox / Nav2 with traversability costmap (Ahmad), victim detection (Ahmad),
ground control station + middleware (Azam), micro-ROS/STM32 hardware interface (Nasir).

Why wheels instead of simulated track links: contact between dozens of track links is slow and unstable in
Gazebo; powered wheels with a visual belt reproduce skid-steer kinematics, slip and terrain interaction well enough
for navigation work.
