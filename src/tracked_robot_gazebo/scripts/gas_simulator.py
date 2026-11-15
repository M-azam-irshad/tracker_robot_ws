#!/usr/bin/env python3
"""Simulated MQ-style gas sensors + hazard classification for the SAR world.

Gazebo has no gas physics, so each gas is modelled as Gaussian plumes with known centres.
The sensor reading = sum of plumes at the gas-sensor position (ground-truth robot pose from Gazebo)
+ baseline, passed through a first-order response lag (real MQ sensors are slow) and noise.

Publishes
  /gas/co_ppm, /gas/ch4_ppm, /gas/h2s_ppm  (std_msgs/Float32)
  /gas/hazard_level   (std_msgs/Int8)   0 SAFE, 1 CAUTION, 2 HAZARDOUS
  /gas/hazard_zone    (std_msgs/String)
  /gas/plume_markers  (visualization_msgs/MarkerArray, frame 'world')  ground-truth debug view
Subscribes
  /robot_ground_truth_pose (tf2_msgs/TFMessage) bridged from Gazebo dynamic_pose
"""
import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

ZONES = ['SAFE', 'CAUTION', 'HAZARDOUS']
COLORS = {'co': (1.0, 0.5, 0.0), 'ch4': (1.0, 0.1, 0.1), 'h2s': (0.8, 0.9, 0.1)}


class GasSimulator(Node):
    def __init__(self):
        super().__init__('gas_simulator')
        d = self.declare_parameter
        self.robot_name = d('robot_name', 'tracked_robot').value
        self.rate = d('publish_rate', 10.0).value
        self.tau = d('response_time_const', 3.0).value
        self.noise_frac = d('noise_fraction', 0.03).value
        self.sensor_dx = d('sensor_offset_x', -0.18).value   # gas sensor position in base_link
        self.sensor_dy = d('sensor_offset_y', 0.0).value
        self.x = d('spawn_x', 0.0).value
        self.y = d('spawn_y', 0.0).value
        self.yaw = d('spawn_yaw', 0.0).value
        self.gases = list(d('gases', ['co', 'ch4', 'h2s']).value)
        self.baseline = list(d('baseline_ppm', [0.0, 2.0, 0.0]).value)
        self.caution = list(d('caution_ppm', [35.0, 5000.0, 10.0]).value)
        self.hazard = list(d('hazard_ppm', [200.0, 25000.0, 50.0]).value)
        self.p_gas = list(d('plume_gas', ['co']).value)
        self.p_x = list(d('plume_x', [0.0]).value)
        self.p_y = list(d('plume_y', [0.0]).value)
        self.p_peak = list(d('plume_peak_ppm', [0.0]).value)
        self.p_sigma = list(d('plume_sigma', [1.0]).value)

        self.filtered = {g: self.baseline[i] for i, g in enumerate(self.gases)}
        self.pub = {g: self.create_publisher(Float32, f'gas/{g}_ppm', 10) for g in self.gases}
        self.level_pub = self.create_publisher(Int8, 'gas/hazard_level', 10)
        self.zone_pub = self.create_publisher(String, 'gas/hazard_zone', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'gas/plume_markers', 1)
        self.create_subscription(TFMessage, 'robot_ground_truth_pose', self.on_pose, 10)
        self.create_timer(1.0 / self.rate, self.step)
        self.create_timer(1.0, self.publish_markers)
        self.last_zone = None
        self.get_logger().info(
            f'Gas simulator: {len(self.p_x)} plumes, gases={self.gases}, robot={self.robot_name}')

    def on_pose(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == self.robot_name:
                q = t.transform.rotation
                self.x = t.transform.translation.x
                self.y = t.transform.translation.y
                self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return

    def concentration(self, gas, sx, sy):
        total = 0.0
        for g, px, py, peak, sig in zip(self.p_gas, self.p_x, self.p_y, self.p_peak, self.p_sigma):
            if g == gas:
                d2 = (sx - px) ** 2 + (sy - py) ** 2
                total += peak * math.exp(-d2 / (2.0 * sig * sig))
        return total

    def step(self):
        dt = 1.0 / self.rate
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        sx = self.x + c * self.sensor_dx - s * self.sensor_dy
        sy = self.y + s * self.sensor_dx + c * self.sensor_dy
        alpha = min(1.0, dt / max(self.tau, 1e-3))
        level = 0
        for i, g in enumerate(self.gases):
            true_val = self.baseline[i] + self.concentration(g, sx, sy)
            self.filtered[g] += alpha * (true_val - self.filtered[g])
            reading = max(0.0, self.filtered[g] * (1.0 + random.gauss(0.0, self.noise_frac)))
            self.pub[g].publish(Float32(data=float(reading)))
            if reading >= self.hazard[i]:
                level = max(level, 2)
            elif reading >= self.caution[i]:
                level = max(level, 1)
        self.level_pub.publish(Int8(data=level))
        self.zone_pub.publish(String(data=ZONES[level]))
        if ZONES[level] != self.last_zone:
            self.get_logger().info(f'Hazard zone -> {ZONES[level]}')
            self.last_zone = ZONES[level]

    def publish_markers(self):
        arr = MarkerArray()
        for i, (g, px, py, sig) in enumerate(zip(self.p_gas, self.p_x, self.p_y, self.p_sigma)):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'gas_plumes'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = px, py, 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 2.0 * sig
            r, gr, b = COLORS.get(g, (1.0, 1.0, 1.0))
            m.color.r, m.color.g, m.color.b, m.color.a = r, gr, b, 0.3
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = GasSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
