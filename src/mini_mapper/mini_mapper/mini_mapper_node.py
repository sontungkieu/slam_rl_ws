#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion


def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def bresenham(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0); dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return points


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class MiniMapper(Node):
    """Tiny occupancy-grid mapper to demonstrate ray casting + log-odds.

    Uses /odom pose, so this is a demo mapper (not full SLAM).
    Publishes /mini_map.
    """

    def __init__(self):
        super().__init__("mini_mapper")
        # self.declare_parameter("use_sim_time", True)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("size_m", 12.0)
        self.declare_parameter("frame_id", "odom")

        self.res = float(self.get_parameter("resolution").value)
        self.size_m = float(self.get_parameter("size_m").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.w = int(self.size_m / self.res)
        self.h = int(self.size_m / self.res)
        self.logodds = np.zeros((self.h, self.w), dtype=np.float32)

        self.L_FREE = -0.4
        self.L_OCC = 0.85
        self.L_MIN, self.L_MAX = -5.0, 5.0

        self.pose: Optional[Pose2D] = None

        self.map_pub = self.create_publisher(OccupancyGrid, "/mini_map", 1)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 50)

        self.timer = self.create_timer(0.5, self.publish_map)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(x / self.res + self.w / 2)
        gy = int(y / self.res + self.h / 2)
        return gx, gy

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = Pose2D(p.x, p.y, yaw_from_quat(q))

    def on_scan(self, msg: LaserScan):
        if self.pose is None:
            return
        self.update_map(self.pose, msg)

    def update_map(self, pose: Pose2D, scan: LaserScan):
        rx, ry, rth = pose.x, pose.y, pose.yaw
        rgx, rgy = self.world_to_grid(rx, ry)

        ang = scan.angle_min
        for r in scan.ranges:
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                ang += scan.angle_increment
                continue

            hit_x = rx + r * math.cos(rth + ang)
            hit_y = ry + r * math.sin(rth + ang)
            hgx, hgy = self.world_to_grid(hit_x, hit_y)

            cells = bresenham(rgx, rgy, hgx, hgy)
            for cx, cy in cells[:-1]:
                if 0 <= cx < self.w and 0 <= cy < self.h:
                    self.logodds[cy, cx] += self.L_FREE
            if 0 <= hgx < self.w and 0 <= hgy < self.h:
                self.logodds[hgy, hgx] += self.L_OCC

            ang += scan.angle_increment

        np.clip(self.logodds, self.L_MIN, self.L_MAX, out=self.logodds)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.info.resolution = self.res
        msg.info.width = self.w
        msg.info.height = self.h
        msg.info.origin.position.x = -self.size_m / 2
        msg.info.origin.position.y = -self.size_m / 2
        msg.info.origin.orientation.w = 1.0

        p = 1.0 - 1.0 / (1.0 + np.exp(self.logodds))
        occ = (p * 100.0).astype(np.int8)
        msg.data = occ.flatten().tolist()

        self.map_pub.publish(msg)


def main():
    rclpy.init()
    node = MiniMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
