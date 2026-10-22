#!/usr/bin/env python3
"""Stand-alone smoke test for neupan_node (no simulator).

Broadcasts identity TF (map->base_link, map->laser_link), feeds a straight
/initial_path and a synthetic /scan, and verifies the node emits /cmd_vel.
Run the node (e.g. via the launch file) before/alongside this script.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


class SmokeDriver(Node):
    def __init__(self):
        super().__init__("neupan_smoke_driver")
        self.cmd_count = 0
        self.last_cmd = None

        self.tf_static = StaticTransformBroadcaster(self)
        self._send_static_tf()

        self.path_pub = self.create_publisher(Path, "/initial_path", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        self.create_timer(0.5, self._pub_path)   # path is latched in planner, repeat a few times
        self.create_timer(0.1, self._pub_scan)
        self.create_timer(5.0, self._finish)

    def _send_static_tf(self):
        tfs = []
        for child in ("base_link", "laser_link"):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            tfs.append(t)
        self.tf_static.sendTransform(tfs)

    def _pub_path(self):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(11):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = i * 0.5  # straight line ahead, 5 m
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _pub_scan(self):
        msg = LaserScan()
        msg.header.frame_id = "laser_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        n = 360
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (n - 1)
        msg.range_min = 0.0
        msg.range_max = 10.0
        # obstacle wall at 3 m to the left/front, free elsewhere
        msg.ranges = [4.0 if (-0.3 < (msg.angle_min + i * msg.angle_increment) < 0.3)
                      else 9.0 for i in range(n)]
        self.scan_pub.publish(msg)

    def _on_cmd(self, msg):
        self.cmd_count += 1
        self.last_cmd = msg

    def _finish(self):
        if self.cmd_count > 0:
            self.get_logger().info(
                f"PASS: received {self.cmd_count} /cmd_vel msgs; "
                f"last v={self.last_cmd.linear.x:.3f} w={self.last_cmd.angular.z:.3f}")
            rclpy.shutdown()
            sys.exit(0)
        else:
            self.get_logger().error("FAIL: no /cmd_vel received")
            rclpy.shutdown()
            sys.exit(1)


def main():
    rclpy.init()
    rclpy.spin(SmokeDriver())


if __name__ == "__main__":
    main()
