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
Each track is modelled as **4 powered wheels** (sprocket, 2 road wheels, idler) that carry the
collision geometry and the drive joints, plus a **visual-only belt** wrapped around them.
This is the standard, numerically stable way to simulate a skid-steer tracked vehicle in Gazebo.

## Replacing primitives with the real CAD
Dimensions are ESTIMATES taken from the NCRA chassis photo. Put the real measurements in
`robot_properties.xacro`, and export STL/DAE meshes into `meshes/visual` and `meshes/collision`.
