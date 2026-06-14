#!/usr/bin/env python3
"""Headless check: robot tracking a SPARSE /initial_path stops at the end.

Integrates /cmd_vel through a diff model and broadcasts TF, publishes a sparse
3-point path (mimics A* + Douglas-Peucker output, 2 m spacing) on /initial_path,
and reports the final resting position and worst overshoot past the endpoint.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class Test(Node):
    def __init__(self):
        super().__init__("path_overshoot_test")
        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.dt = 0.02
        self.t = 0.0
        self.goal_x = 4.0
        self.max_x = 0.0
        self.sent = False

        self.bc = TransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.path_pub = self.create_publisher(Path, "/initial_path", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_timer(self.dt, self._step)
        self.create_timer(0.1, self._pub_scan)

    def _on_cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z

    def _bcast(self):
        now = self.get_clock().now().to_msg()
        for ch in ("base_footprint", "base_link"):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "map"
            t.child_frame_id = ch
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = math.sin(self.th / 2)
            t.transform.rotation.w = math.cos(self.th / 2)
            self.bc.sendTransform(t)

    def _pub_path(self):
        p = Path()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        for wx in (0.0, 2.0, 4.0):  # sparse, 2 m spacing
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = wx
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.path_pub.publish(p)

    def _step(self):
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt
        self.th += self.w * self.dt
        self._bcast()
        self.max_x = max(self.max_x, self.x)
        self.t += self.dt
        if not self.sent and self.t > 1.0:
            self._pub_path()
            self.sent = True
            self.get_logger().info("published sparse /initial_path [0, 2, 4]")
        if self.sent and int(self.t * 50) % 50 == 0:
            self.get_logger().info(f"t={self.t:4.1f} x={self.x:+.3f} v={self.v:+.2f}")
        if self.t > 18.0:
            overshoot = self.max_x - self.goal_x
            stopped = abs(self.v) < 0.02
            final_err = abs(self.x - self.goal_x)
            ok = final_err < 0.25 and overshoot < 0.25
            self.get_logger().info(
                f"RESULT final_x={self.x:.3f} max_x={self.max_x:.3f} "
                f"overshoot={overshoot:+.3f} final_err={final_err:.3f} stopped={stopped}")
            self.get_logger().info("PASS" if ok else "FAIL: overshoot/err too large")
            rclpy.shutdown(); sys.exit(0 if ok else 1)

    def _pub_scan(self):
        m = LaserScan()
        m.header.frame_id = "base_footprint"
        m.header.stamp = self.get_clock().now().to_msg()
        m.angle_min, m.angle_max = -math.pi, math.pi
        n = 180
        m.angle_increment = (m.angle_max - m.angle_min) / (n - 1)
        m.range_min, m.range_max = 0.45, 10.0
        m.ranges = [9.0] * n
        self.scan_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Test())


if __name__ == "__main__":
    main()
