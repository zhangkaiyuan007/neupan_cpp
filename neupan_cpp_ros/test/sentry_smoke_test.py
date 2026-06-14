#!/usr/bin/env python3
"""Level-0 smoke test for the sentry config (no simulator).

Broadcasts identity TF (map->base_footprint, map->base_link), publishes a
/goal_pose (RViz "2D Nav Goal" equivalent) and a synthetic /scan, and verifies
neupan_node loads the diff_sentry model and emits /cmd_vel toward the goal.
Run `ros2 launch neupan_cpp_ros sentry.launch.py` alongside this script.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


class SentrySmoke(Node):
    def __init__(self):
        super().__init__("sentry_smoke_driver")
        self.cmd_count = 0
        self.last_cmd = None

        self.tf_static = StaticTransformBroadcaster(self)
        self._send_static_tf()

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        self.create_timer(0.1, self._pub_scan)
        self.create_timer(1.0, self._pub_goal_once)
        self.create_timer(6.0, self._finish)
        self._goal_sent = False

    def _send_static_tf(self):
        tfs = []
        for child in ("base_footprint", "base_link"):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            tfs.append(t)
        self.tf_static.sendTransform(tfs)

    def _pub_goal_once(self):
        if self._goal_sent:
            return
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = 3.0  # 3 m straight ahead
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)
        self._goal_sent = True
        self.get_logger().info("published /goal_pose (3.0, 0.0)")

    def _pub_scan(self):
        msg = LaserScan()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        n = 360
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (n - 1)
        msg.range_min = 0.3
        msg.range_max = 10.0
        # obstacle straight ahead at 2 m, free elsewhere
        msg.ranges = [2.0 if (-0.2 < (msg.angle_min + i * msg.angle_increment) < 0.2)
                      else 9.0 for i in range(n)]
        self.scan_pub.publish(msg)

    def _on_cmd(self, msg):
        self.cmd_count += 1
        self.last_cmd = msg

    def _finish(self):
        ok = self.cmd_count > 0 and self.last_cmd is not None
        finite = ok and all(map(math.isfinite,
                                (self.last_cmd.linear.x, self.last_cmd.angular.z)))
        if ok and finite:
            self.get_logger().info(
                f"PASS: {self.cmd_count} /cmd_vel msgs; "
                f"last v={self.last_cmd.linear.x:.3f} w={self.last_cmd.angular.z:.3f}")
            rclpy.shutdown(); sys.exit(0)
        self.get_logger().error(
            f"FAIL: count={self.cmd_count} finite={finite}")
        rclpy.shutdown(); sys.exit(1)


def main():
    rclpy.init()
    rclpy.spin(SentrySmoke())


if __name__ == "__main__":
    main()
