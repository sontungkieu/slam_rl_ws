#!/usr/bin/env python3
import random
from collections import deque, defaultdict
from typing import Optional, Tuple, Dict, List

import psutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


def find_pid_by_cmd_contains(substr: str) -> Optional[int]:
    s = substr.lower()
    for p in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
        try:
            name = (p.info.get("name") or "").lower()
            cmd = " ".join(p.info.get("cmdline") or []).lower()
            if s in name or s in cmd:
                return int(p.info["pid"])
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return None


def safe_set_nice(pid: int, nice_value: int, logger):
    try:
        psutil.Process(pid).nice(nice_value)
        logger.info(f"nice({pid})={nice_value}")
    except psutil.AccessDenied:
        logger.warn(f"no permission to set nice for PID={pid} (try sudo if you really need).")
    except psutil.NoSuchProcess:
        logger.warn(f"PID={pid} not found.")


class RLResourceManager(Node):
    """Minimal online Q-learning for resource management.

    Observations (state):
      - scan_ok (1/0) from /scan rate
      - map_ok (1/0) from /map period
      - cpu_hi (1/0) from total CPU percent

    Actions:
      0 BALANCED
      1 PRIORITIZE_SLAM (deprioritize cpu_hog + rviz2 + explorer)
      2 PRIORITIZE_VIZ  (keep rviz responsive; still limit cpu_hog)

    This gives you a clean OS story: RL changes process priority (nice) to protect SLAM.
    """

    def __init__(self):
        super().__init__("rl_resource_manager")
        # self.declare_parameter("use_sim_time", True)
        self.declare_parameter("step_s", 2.0)

        self.declare_parameter("min_scan_hz", 5.0)
        self.declare_parameter("max_map_period_s", 1.5)
        self.declare_parameter("cpu_high_pct", 75.0)

        self.declare_parameter("eps", 0.15)
        self.declare_parameter("alpha", 0.25)
        self.declare_parameter("gamma", 0.85)

        self.scan_times = deque(maxlen=200)
        self.map_times = deque(maxlen=60)

        self.create_subscription(LaserScan, "/scan", self.on_scan, 50)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)

        self.Q: Dict[Tuple[int,int,int], List[float]] = defaultdict(lambda: [0.0, 0.0, 0.0])
        self.prev_state: Optional[Tuple[int,int,int]] = None
        self.prev_action: Optional[int] = None

        self.pid_hog = None
        self.pid_rviz = None
        self.pid_explorer = None

        self.create_timer(float(self.get_parameter("step_s").value), self.step)
        self.get_logger().info("rl_resource_manager started. Waiting /scan + /map ...")

    def on_scan(self, msg: LaserScan):
        self.scan_times.append(self.get_clock().now().nanoseconds / 1e9)

    def on_map(self, msg: OccupancyGrid):
        self.map_times.append(self.get_clock().now().nanoseconds / 1e9)

    def _rate_hz(self, times: deque) -> float:
        if len(times) < 2:
            return 0.0
        dt = times[-1] - times[0]
        return (len(times)-1) / dt if dt > 1e-6 else 0.0

    def _period_s(self, times: deque) -> float:
        if len(times) < 2:
            return float("inf")
        return times[-1] - times[-2]

    def _state(self) -> Tuple[int,int,int]:
        scan_hz = self._rate_hz(self.scan_times)
        map_period = self._period_s(self.map_times)
        cpu_total = psutil.cpu_percent(interval=None)

        min_scan = float(self.get_parameter("min_scan_hz").value)
        max_map_p = float(self.get_parameter("max_map_period_s").value)
        cpu_high = float(self.get_parameter("cpu_high_pct").value)

        scan_ok = 1 if scan_hz >= min_scan else 0
        map_ok = 1 if map_period <= max_map_p else 0
        cpu_hi = 1 if cpu_total >= cpu_high else 0

        self.get_logger().info(f"scan={scan_hz:.1f}Hz mapP={map_period:.2f}s cpu={cpu_total:.0f}% -> {scan_ok,map_ok,cpu_hi}")
        return (scan_ok, map_ok, cpu_hi)

    def _reward(self, state: Tuple[int,int,int]) -> float:
        scan_ok, map_ok, cpu_hi = state
        r = (1.0 if scan_ok else -1.0) + (1.0 if map_ok else -1.2) + (-0.3 if cpu_hi else 0.1)
        return r

    def _choose_action(self, state) -> int:
        eps = float(self.get_parameter("eps").value)
        if random.random() < eps:
            return random.randint(0, 2)
        q = self.Q[state]
        return int(max(range(3), key=lambda i: q[i]))

    def _apply_action(self, action: int):
        # discover pids lazily
        if self.pid_hog is None:
            self.pid_hog = find_pid_by_cmd_contains("cpu_hog")
        if self.pid_rviz is None:
            self.pid_rviz = find_pid_by_cmd_contains("rviz2")
        if self.pid_explorer is None:
            self.pid_explorer = find_pid_by_cmd_contains("reactive_explorer")

        if action == 0:
            if self.pid_hog: safe_set_nice(self.pid_hog, 0, self.get_logger())
            if self.pid_rviz: safe_set_nice(self.pid_rviz, 0, self.get_logger())
            if self.pid_explorer: safe_set_nice(self.pid_explorer, 0, self.get_logger())
            self.get_logger().info("action 0 BALANCED")
        elif action == 1:
            if self.pid_hog: safe_set_nice(self.pid_hog, 15, self.get_logger())
            if self.pid_rviz: safe_set_nice(self.pid_rviz, 10, self.get_logger())
            if self.pid_explorer: safe_set_nice(self.pid_explorer, 5, self.get_logger())
            self.get_logger().info("action 1 PRIORITIZE_SLAM")
        else:
            if self.pid_hog: safe_set_nice(self.pid_hog, 8, self.get_logger())
            if self.pid_rviz: safe_set_nice(self.pid_rviz, 0, self.get_logger())
            if self.pid_explorer: safe_set_nice(self.pid_explorer, 0, self.get_logger())
            self.get_logger().info("action 2 PRIORITIZE_VIZ")

    def step(self):
        state = self._state()
        reward = self._reward(state)

        if self.prev_state is not None and self.prev_action is not None:
            alpha = float(self.get_parameter("alpha").value)
            gamma = float(self.get_parameter("gamma").value)
            qsa = self.Q[self.prev_state][self.prev_action]
            self.Q[self.prev_state][self.prev_action] = qsa + alpha * (reward + gamma * max(self.Q[state]) - qsa)

        action = self._choose_action(state)
        self._apply_action(action)

        self.prev_state = state
        self.prev_action = action

def main():
    rclpy.init()
    node = RLResourceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
