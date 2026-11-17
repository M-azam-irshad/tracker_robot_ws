# TF frames (REP-103/105)
```
map -> odom -> base_footprint -> base_link -+- imu_link
(world -> map static, debug only)           |- lidar_mount_link -> lidar_link
                                            |- kinect_mount_link -> camera_link
                                            |- gas_mount_link -> gas_sensor_link
                                            |- microphone_link
                                            |- {left,right}_track_belt   (visual + ramp collisions, fixed)
                                            |- {left,right}_sprocket_wheel
                                            `- {left,right}_{front,rear}_roller_wheel
```
* base_link = geometric centre of the chassis box, 0.205 m above ground; base_footprint is on the ground.
* x forward, y left, z up. Positive wheel rotation about +Y moves the robot forward.
* `odom -> base_footprint` is published by diff_drive_controller. `map -> odom` will come from SLAM.
