# Topics / services
| topic | type | source |
|---|---|---|
| /clock | rosgraph_msgs/Clock | Gazebo |
| /scan | sensor_msgs/LaserScan | lidar_link, 10 Hz |
| /imu/data | sensor_msgs/Imu | imu_link, 100 Hz |
| /camera/image, /camera/depth_image, /camera/camera_info, /camera/points | Image / CameraInfo / PointCloud2 | camera_link, 15 Hz |
| /joint_states | JointState | joint_state_broadcaster |
| /diff_drive_controller/odom, /odom | Odometry | diff_drive_controller (+ relay) |
| /cmd_vel | Twist | manual teleop (highest priority) |
| /cmd_vel_nav | Twist | Nav2 / autonomy |
| /diff_drive_controller/cmd_vel | TwistStamped | track_drive_controller output |
| /emergency_stop | std_srvs/SetBool | service |
| /gas/co_ppm, /gas/ch4_ppm, /gas/h2s_ppm | Float32 | gas_simulator |
| /gas/hazard_level (0/1/2), /gas/hazard_zone | Int8 / String | gas_simulator |
| /gas/plume_markers | MarkerArray | ground-truth debug |
| /robot_ground_truth_pose | tf2_msgs/TFMessage | Gazebo (evaluation only) |
