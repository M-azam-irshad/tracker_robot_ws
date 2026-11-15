# Component inventory (simulated vs real)
| component | simulated as | status |
|---|---|---|
| NCRA chassis | box 0.50x0.32x0.14, 18 kg | estimate - measure |
| DC motors + tracks | 8 velocity-driven wheels, belt visual | estimate |
| RPLIDAR | gpu_lidar 360 deg, 0.15-12 m, 10 Hz | model/spec to confirm |
| Kinect RGB-D | rgbd_camera 320x240, 57 deg, 0.4-4.5 m | ok |
| IMU | gz imu 100 Hz with noise | ok |
| Gas sensors (MQ) | gas_simulator.py | thresholds placeholder |
| Microphone array | frame only (`microphone_link`) | TODO sound-source sim |
| Raspberry Pi 5 / STM32 | not simulated | - |
