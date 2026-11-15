# Sensor configuration
Edit sensors in `urdf/robot/sensors.xacro`; matching ROS topics in `tracked_robot_gazebo/config/bridge.yaml`.
Gas model: Gaussian plumes (`config/sensors.yaml`) + first-order lag (3 s) + 3% noise; hazard level = max over gases.
Plume centres must match `GAS_SOURCES` in `scripts/generate_worlds.py` (visible as translucent spheres).
Victims: `victim_1..3` static models; world coordinates in `generate_worlds.py` (VICTIMS).
