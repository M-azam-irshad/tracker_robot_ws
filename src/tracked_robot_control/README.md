# tracked_robot_control

* `config/controllers.yaml`  - controller_manager (loaded by the gz_ros2_control plugin)
* `config/diff_drive_controller.yaml` - skid-steer diff-drive parameters (loaded by the spawner)
* `config/joint_limits.yaml` - speed/acceleration limits for `track_drive_controller`
* `track_drive_controller` (C++ node) - command arbitration + safety layer in front of the controller

## Command chain
```
teleop (/cmd_vel, Twist) ──┐  priority
nav2   (/cmd_vel_nav)    ──┴─> track_drive_controller ──> /diff_drive_controller/cmd_vel (TwistStamped)
                               * manual overrides autonomy while teleop commands are fresh
                               * watchdog timeout -> zero velocity
                               * acceleration ramp + speed clamp
                               * /emergency_stop (std_srvs/SetBool)
                               * relays /diff_drive_controller/odom -> /odom
```

Emergency stop:
```bash
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"   # engage
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: false}"  # release
```
