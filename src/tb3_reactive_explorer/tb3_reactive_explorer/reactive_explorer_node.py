#!/usr/bin/env python3
import math
import random
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan


def finite_min(vals):
    m = None
    for v in vals:
        if math.isfinite(v):
            m = v if m is None else min(m, v)
    return m if m is not None else float("inf")


class ReactiveExplorer(Node):
    """/scan -> /cmd_vel reactive exploration (obstacle avoidance + random walk)."""

    def __init__(self):
        super().__init__("reactive_explorer")

        self.declare_parameter("forward_speed", 0.18)
        self.declare_parameter("turn_speed", 0.8)
        self.declare_parameter("avoid_front", 0.45)
        self.declare_parameter("avoid_side", 0.35)
        self.declare_parameter("wander_turn_every_s", 8.0)
        self.declare_parameter("wander_turn_duration_s", 1.2)

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )

        self.last_scan: Optional[LaserScan] = None
        self.wander_until = self.get_clock().now()
        self.next_wander = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.tick)  # 10Hz

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def _sector_min(self, scan: LaserScan, ang_from: float, ang_to: float) -> float:
        if ang_from > ang_to:
            ang_from, ang_to = ang_to, ang_from
        a0 = max(ang_from, scan.angle_min)
        a1 = min(ang_to, scan.angle_max)
        if a0 >= a1:
            return float("inf")
        i0 = int((a0 - scan.angle_min) / scan.angle_increment)
        i1 = int((a1 - scan.angle_min) / scan.angle_increment)
        i0 = max(0, min(i0, len(scan.ranges) - 1))
        i1 = max(0, min(i1, len(scan.ranges) - 1))
        if i1 < i0:
            i0, i1 = i1, i0
        return finite_min(scan.ranges[i0:i1 + 1])

    def _make_cmd(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        return msg

    def tick(self):
        if self.last_scan is None:
            return
        scan = self.last_scan
        now = self.get_clock().now()

        front = self._sector_min(scan, math.radians(-20), math.radians(20))
        left  = self._sector_min(scan, math.radians(20), math.radians(90))
        right = self._sector_min(scan, math.radians(-90), math.radians(-20))

        # Clamp non-finite sector mins to a sane fallback (range_max usually ~3.5m in TB3 sim)
        fallback = scan.range_max if math.isfinite(scan.range_max) else 3.5
        if not math.isfinite(front): front = fallback
        if not math.isfinite(left):  left  = fallback
        if not math.isfinite(right): right = fallback

        v = float(self.get_parameter("forward_speed").value)
        turn_speed = float(self.get_parameter("turn_speed").value)
        avoid_front = float(self.get_parameter("avoid_front").value)
        avoid_side = float(self.get_parameter("avoid_side").value)

        # Occasionally start a gentle arc-turn
        if now >= self.next_wander:
            every = float(self.get_parameter("wander_turn_every_s").value)
            self.next_wander = now + rclpy.time.Duration(seconds=every)
            if random.random() < 0.25:
                dur = float(self.get_parameter("wander_turn_duration_s").value)
                self.wander_until = now + rclpy.time.Duration(seconds=dur)

        t = self._make_cmd()

        if front < avoid_front:
            t.twist.linear.x = 0.0
            t.twist.angular.z = turn_speed if left > right else -turn_speed
        else:
            t.twist.linear.x = v
            if left < avoid_side and right >= avoid_side:
                t.twist.angular.z = -0.6 * turn_speed
            elif right < avoid_side and left >= avoid_side:
                t.twist.angular.z = 0.6 * turn_speed
            else:
                if now < self.wander_until:
                    t.twist.angular.z = 0.35 * turn_speed
                else:
                    denom = max(left, right, 1e-3)
                    diff = right - left
                    t.twist.angular.z = 0.25 * turn_speed * (diff / denom)

        # Final safety: never publish NaN/inf
        if not math.isfinite(t.twist.angular.z):
            t.twist.angular.z = 0.0
        if not math.isfinite(t.twist.linear.x):
            t.twist.linear.x = 0.0

        self.cmd_pub.publish(t)

    def stop(self):
        t = self._make_cmd()
        t.twist.linear.x = 0.0
        t.twist.angular.z = 0.0
        self.cmd_pub.publish(t)


def main():
    rclpy.init()
    node = ReactiveExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
