# tracked_robot_description

Xacro/URDF model of the tracked SAR rover.

```
urdf/tracked_robot.urdf.xacro      <- entry point (xacro arg: use_sim:=true|false)
urdf/robot/robot_properties.xacro  <- ALL dimensions / masses live here
urdf/robot/chassis.xacro           <- base_footprint, base_link
urdf/robot/track_components.xacro  <- wheel + track-belt macros
urdf/robot/left_track.xacro        <- instantiates the left track
urdf/robot/right_track.xacro       <- instantiates the right track
urdf/robot/sensor_mounts.xacro     <- masts / brackets
urdf/robot/sensors.xacro           <- sensor frames + Gazebo sensor plugins
urdf/robot/inertials.xacro         <- inertia macros
urdf/robot/materials.xacro
urdf/ros2_control/ros2_control.xacro
```

View in RViz (no Gazebo):

```bash
ros2 launch tracked_robot_description display.launch.py
```

## Modelling approach
The real track is a raised-sprocket triangle (sprocket 18.8 cm above ground, two 8.45 cm rollers on the ground).
Per side: 2 powered rollers (traction), 1 spinning visual sprocket, and a belt link carrying the STEP-derived visual
mesh plus passive ramp/skid collisions. See `docs/diagrams/track_collision_profile.png`.

## Replacing primitives with the real CAD
Dimensions come from your measurement table and the STEP files (tagged [T]/[S]/[E] in `robot_properties.xacro`).
Meshes are procedural (`scripts/generate_meshes.py`); swap in decimated CAD STLs with the same names/origins.
