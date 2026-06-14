#!/usr/bin/env python3
"""Headless closed-loop check for neupan_node (no gazebo).

Integrates /cmd_vel through a diff-drive model and broadcasts the resulting
map->base_footprint / map->base_link TF, so NeuPAN sees the robot actually
moving. Publishes a goal and an (empty) scan, then logs the trajectory and the
command stream to reveal spin-in-place / non-convergence.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class ClosedLoop(Node):
    def __init__(self, gx, gy):
        super().__init__("closed_loop_test")
        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.gx, self.gy = gx, gy
        self.dt = 0.02
        self.t = 0.0
        self.goal_sent = False

        self.bc = TransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        self.create_timer(self.dt, self._step)
        self.create_timer(0.1, self._pub_scan)

    def _on_cmd(self, msg):
        self.v, self.w = msg.linear.x, msg.angular.z

    def _bcast(self):
        now = self.get_clock().now().to_msg()
        for child in ("base_footprint", "base_link"):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "map"
            t.child_frame_id = child
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = math.sin(self.th / 2)
            t.transform.rotation.w = math.cos(self.th / 2)
            self.bc.sendTransform(t)

    def _step(self):
        # integrate diff-drive
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt
        self.th = math.atan2(math.sin(self.th + self.w * self.dt),
                             math.cos(self.th + self.w * self.dt))
        self._bcast()
        self.t += self.dt

        if not self.goal_sent and self.t > 1.0:
            g = PoseStamped()
            g.header.frame_id = "map"
            g.header.stamp = self.get_clock().now().to_msg()
            g.pose.position.x = float(self.gx)
            g.pose.position.y = float(self.gy)
            g.pose.orientation.w = 1.0
            self.goal_pub.publish(g)
            self.goal_sent = True
            self.get_logger().info(f"goal ({self.gx},{self.gy}) sent")

        if int(self.t * 50) % 25 == 0:  # ~2 Hz log (also before goal)
            d = math.hypot(self.gx - self.x, self.gy - self.y)
            tag = "" if self.goal_sent else " [PRE-GOAL]"
            self.get_logger().info(
                f"t={self.t:4.1f} pose=({self.x:+.2f},{self.y:+.2f},{math.degrees(self.th):+6.1f}deg) "
                f"cmd=(v{self.v:+.2f},w{self.w:+.2f}) dist={d:.2f}{tag}")

        if self.goal_sent and math.hypot(self.gx - self.x, self.gy - self.y) < 0.2:
            self.get_logger().info(f"ARRIVED at t={self.t:.1f}")
            rclpy.shutdown(); sys.exit(0)

        if self.t > 25.0:
            self.get_logger().error("TIMEOUT (did not arrive)")
            rclpy.shutdown(); sys.exit(1)

    def _pub_scan(self):
        msg = LaserScan()
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min, msg.angle_max = -math.pi, math.pi
        n = 360
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (n - 1)
        msg.range_min, msg.range_max = 0.45, 10.0
        msg.ranges = [9.0] * n  # open field, pure tracking
        self.scan_pub.publish(msg)


def main():
    gx = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    gy = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    rclpy.init()
    rclpy.spin(ClosedLoop(gx, gy))


if __name__ == "__main__":
    main()
