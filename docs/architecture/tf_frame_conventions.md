# TF frames (REP-103/105)
```
map -> odom -> base_footprint -> base_link -+- imu_link
(world -> map static, debug only)           |- lidar_mount_link -> lidar_link
                                            |- kinect_mount_link -> camera_link
                                            |- gas_mount_link -> gas_sensor_link
                                            |- microphone_link
                                            |- left/right_track_belt (visual)
                                            `- {left,right}_{sprocket,road_1,road_2,idler}_wheel
```
* x forward, y left, z up. Positive wheel rotation about +Y moves the robot forward.
* `odom -> base_footprint` is published by diff_drive_controller. `map -> odom` will come from SLAM.
* `camera_link` is x-forward (not an optical frame); Gazebo point clouds are published in it.
