#!/usr/bin/env python3
"""Headless test for astar_global_node.

Publishes a synthetic /map with a wall (gap at the top), broadcasts TF, sends a
goal behind the wall, and checks /initial_path detours around it.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf2_ros import StaticTransformBroadcaster


class AStarTest(Node):
    def __init__(self):
        super().__init__("astar_test")
        self.W, self.H, self.res = 100, 100, 0.1
        self.ox, self.oy = -5.0, -5.0
        self.got_path = None

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.create_subscription(Path, "/initial_path", self._on_path, 10)

        self.tf = StaticTransformBroadcaster(self)
        self._send_tf()
        self._publish_map()
        self.create_timer(1.0, self._send_goal_once)
        self.create_timer(4.0, self._finish)
        self._goal_sent = False

    def _send_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_footprint"
        t.transform.rotation.w = 1.0  # robot at origin
        self.tf.sendTransform([t])

    def _publish_map(self):
        m = OccupancyGrid()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = self.res
        m.info.width = self.W
        m.info.height = self.H
        m.info.origin.position.x = self.ox
        m.info.origin.position.y = self.oy
        m.info.origin.orientation.w = 1.0
        data = [0] * (self.W * self.H)
        # wall at world x in [2.0,2.3], y in [-5, 1.5] (gap above y=1.5)
        for cy in range(self.H):
            wy = self.oy + (cy + 0.5) * self.res
            if wy > 1.5:
                continue
            for cx in range(self.W):
                wx = self.ox + (cx + 0.5) * self.res
                if 2.0 <= wx <= 2.3:
                    data[cy * self.W + cx] = 100
        m.data = data
        self.map_pub.publish(m)
        self.get_logger().info("published synthetic /map with wall + gap")

    def _send_goal_once(self):
        if self._goal_sent:
            return
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = 4.0  # behind the wall
        g.pose.position.y = 0.0
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)
        self._goal_sent = True
        self.get_logger().info("sent goal (4.0, 0.0)")

    def _on_path(self, msg):
        self.got_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _finish(self):
        if not self.got_path:
            self.get_logger().error("FAIL: no /initial_path received")
            rclpy.shutdown(); sys.exit(1)
        xs = [p[0] for p in self.got_path]
        ys = [p[1] for p in self.got_path]
        max_y = max(ys)
        start_ok = math.hypot(xs[0], ys[0]) < 1.0
        end_ok = math.hypot(xs[-1] - 4.0, ys[-1]) < 0.6
        # any waypoint sitting inside the wall band?
        in_wall = any(2.0 <= x <= 2.3 and y <= 1.5 for x, y in self.got_path)
        detoured = max_y > 1.4  # had to climb to the gap
        ok = start_ok and end_ok and detoured and not in_wall
        self.get_logger().info(
            f"path: {len(self.got_path)} wpts, start={self.got_path[0]}, "
            f"end={self.got_path[-1]}, max_y={max_y:.2f}, in_wall={in_wall}")
        self.get_logger().info("PASS" if ok else "FAIL: path did not detour cleanly")
        rclpy.shutdown(); sys.exit(0 if ok else 1)


def main():
    rclpy.init()
    rclpy.spin(AStarTest())


if __name__ == "__main__":
    main()
