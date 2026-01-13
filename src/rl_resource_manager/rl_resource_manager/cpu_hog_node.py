#!/usr/bin/env python3
import os
import time
import math
import rclpy
from rclpy.node import Node

class CpuHog(Node):
    def __init__(self):
        super().__init__("cpu_hog")
        # self.declare_parameter("use_sim_time", True)
        self.declare_parameter("load", 0.85)    # 0..1
        self.declare_parameter("cycle_ms", 50)  # ms

        self._x = 0.0001
        self.create_timer(0.05, self.tick)  # 20 Hz
        self.get_logger().info(f"cpu_hog PID={os.getpid()} (use this to create contention).")

    def tick(self):
        load = max(0.0, min(1.0, float(self.get_parameter("load").value)))
        cycle_ms = int(self.get_parameter("cycle_ms").value)
        busy_s = load * (cycle_ms / 1000.0)
        idle_s = (cycle_ms / 1000.0) - busy_s

        t0 = time.perf_counter()
        while (time.perf_counter() - t0) < busy_s:
            self._x = math.sin(self._x) + 1.000001

        if idle_s > 0:
            time.sleep(idle_s)

def main():
    rclpy.init()
    node = CpuHog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
