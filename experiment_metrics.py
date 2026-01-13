#!/usr/bin/env python3
import argparse
import csv
import math
import os
import re
import signal
import subprocess
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

# Optional: psutil for better CPU metrics (fallback to `ps` if not installed)
try:
    import psutil  # type: ignore
except Exception:
    psutil = None


def now_mono() -> float:
    return time.monotonic()


def parse_loads(s: str) -> List[float]:
    loads = []
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        v = float(part)
        if v < 0.0 or v > 1.0:
            raise ValueError(f"load must be in [0,1], got {v}")
        loads.append(v)
    if not loads:
        raise ValueError("loads list is empty")
    return loads


def safe_float(x: float) -> float:
    return x if math.isfinite(x) else float("nan")


def ps_cpu_percent(pid: int) -> float:
    """Fallback CPU% via `ps` if psutil not available."""
    try:
        out = subprocess.check_output(["ps", "-p", str(pid), "-o", "%cpu="], text=True).strip()
        return float(out) if out else 0.0
    except Exception:
        return float("nan")


def find_pid_by_regex(pattern: str) -> Optional[int]:
    """Find first PID whose cmdline matches regex (best-effort)."""
    try:
        out = subprocess.check_output(["ps", "-eo", "pid,args"], text=True)
        rx = re.compile(pattern)
        for line in out.splitlines()[1:]:
            line = line.strip()
            if not line:
                continue
            pid_str, args = line.split(None, 1)
            if rx.search(args):
                return int(pid_str)
    except Exception:
        pass
    return None

def pgrep_one(pattern: str) -> Optional[int]:
    """Return one PID matching pattern via pgrep -f (best-effort)."""
    try:
        out = subprocess.check_output(["pgrep", "-f", pattern], text=True).strip().splitlines()
        if not out:
            return None
        # pick the smallest PID (usually the oldest)
        pids = sorted(int(x) for x in out if x.strip().isdigit())
        return pids[0] if pids else None
    except Exception:
        return None


@dataclass
class HogProc:
    popen: subprocess.Popen
    load: float
    start_t: float  # monotonic seconds
    name: str


class RateTracker:
    def __init__(self, window_s: float):
        self.window_s = window_s
        self.times: Deque[float] = deque()

    def add(self, t: float):
        self.times.append(t)
        self._prune(t)

    def _prune(self, t_now: float):
        cutoff = t_now - self.window_s
        while self.times and self.times[0] < cutoff:
            self.times.popleft()

    def rate_hz(self, t_now: float) -> float:
        self._prune(t_now)
        if len(self.times) < 2:
            return 0.0
        dt = self.times[-1] - self.times[0]
        if dt <= 1e-9:
            return 0.0
        return (len(self.times) - 1) / dt


class ExperimentNode(Node):
    def __init__(
        self,
        loads: List[float],
        start_after_s: float,
        step_s: float,
        duration_s: float,
        log_period_s: float,
        window_s: float,
        out_csv: str,
        out_png: str,
        slam_regex: str,
        bridge_regex: str,
        dry_run: bool,
    ):
        super().__init__("metrics_runner")
        # Use sim time if available
        try:
            self.declare_parameter("use_sim_time", True)
        except Exception:
            pass

        self.loads = loads
        self.start_after_s = start_after_s
        self.step_s = step_s
        self.duration_s = duration_s
        self.log_period_s = log_period_s
        self.window_s = window_s
        self.out_csv = out_csv
        self.out_png = out_png
        self.slam_regex = slam_regex
        self.bridge_regex = bridge_regex
        self.dry_run = dry_run

        # Subscriptions
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.sub_map = self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)

        # Track rates based on arrival times
        self.scan_rate = RateTracker(window_s)
        self.map_rate = RateTracker(window_s)

        self.last_scan_stamp = None
        self.last_map_stamp = None

        self.t0 = now_mono()
        self.events: List[Tuple[float, str]] = []  # (t_rel, label)
        self.hogs: List[HogProc] = []
        self.next_hog_idx = 0

        # CSV
        os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
        self.csv_f = open(out_csv, "w", newline="", encoding="utf-8")
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow([
            "t_rel_s",
            "sim_time_s",
            "scan_hz",
            "map_hz",
            "scan_age_s",
            "map_age_s",
            "cpu_slam_percent",
            "cpu_bridge_percent",
            "cpu_hog_total_percent",
            "hog_count",
        ])

        # In-memory for plotting
        self.rows = []
        # ---- CPU process cache ----
        self._proc_cache = {}  # pid -> psutil.Process
        self._slam_pid: Optional[int] = None
        self._bridge_pid: Optional[int] = None
        self._next_pid_scan_t = 0.0
        self._pid_scan_every_s = 2.0  # scan PID every 2s

        # debug log
        self._last_debug_log_t = 0.0
        self._debug_every_s = 5.0

        # Prime psutil cpu_percent (if present)
        if psutil is not None:
            psutil.cpu_percent(interval=None)

        self.timer = self.create_timer(self.log_period_s, self.tick)

    def on_scan(self, msg: LaserScan):
        t = now_mono()
        self.scan_rate.add(t)
        self.last_scan_stamp = msg.header.stamp

    def on_map(self, msg: OccupancyGrid):
        t = now_mono()
        self.map_rate.add(t)
        self.last_map_stamp = msg.header.stamp

    def _ros_time_sec(self) -> float:
        # sim time if /clock exists; else wall time
        return self.get_clock().now().nanoseconds / 1e9

    def _stamp_to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _age_sec(self, stamp) -> float:
        if stamp is None:
            return float("nan")
        return self._ros_time_sec() - self._stamp_to_sec(stamp)

    def _start_hog(self, load: float, idx: int):
        name = f"cpu_hog_{idx}_{int(load*100):02d}"
        cmd = [
            "ros2", "run", "rl_resource_manager", "cpu_hog",
            "--ros-args",
            "-p", f"load:={load}",
            "-r", f"__node:={name}",
        ]
        if self.dry_run:
            self.get_logger().info(f"[DRY RUN] would start: {' '.join(cmd)}")
            self.events.append((now_mono() - self.t0, f"hog {idx} load={load}"))
            return

        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.hogs.append(HogProc(popen=p, load=load, start_t=now_mono(), name=name))
        self.events.append((now_mono() - self.t0, f"hog {idx} load={load}"))
        self.get_logger().info(f"Started {name} (pid={p.pid}) load={load}")

    def _cpu_of_pid(self, pid: Optional[int], label: str = "") -> float:
        if pid is None:
            return float("nan")

        # psutil path (recommended)
        if psutil is not None:
            try:
                proc = self._proc_cache.get(pid)
                if proc is None:
                    proc = psutil.Process(pid)
                    # prime so that next call returns non-zero
                    proc.cpu_percent(interval=None)
                    self._proc_cache[pid] = proc
                    self.get_logger().info(f"CPU monitor attached: {label} pid={pid}")
                    return float("nan")  # first sample after attach is meaningless
                return proc.cpu_percent(interval=None)
            except psutil.NoSuchProcess:
                # process died, drop from cache
                self._proc_cache.pop(pid, None)
                return float("nan")
            except Exception:
                return float("nan")

        # fallback via `ps`
        return ps_cpu_percent(pid)


    def _cpu_total_hogs(self) -> float:
        total = 0.0
        for h in self.hogs:
            v = self._cpu_of_pid(h.popen.pid, label=h.name)
            if math.isfinite(v):
                total += v
        return total


    def tick(self):
        t_now = now_mono()
        t_rel = t_now - self.t0

        # Schedule hog starts
        while self.next_hog_idx < len(self.loads):
            start_t = self.start_after_s + self.next_hog_idx * self.step_s
            if t_rel + 1e-9 >= start_t:
                self._start_hog(self.loads[self.next_hog_idx], self.next_hog_idx + 1)
                self.next_hog_idx += 1
            else:
                break
        # Refresh PIDs every few seconds (avoid heavy ps scanning every tick)
        if t_rel >= self._next_pid_scan_t:
            self._next_pid_scan_t = t_rel + self._pid_scan_every_s

            if self._slam_pid is None:
                self._slam_pid = pgrep_one(self.slam_regex)
                if self._slam_pid is not None:
                    self.get_logger().info(f"Found slam_toolbox pid={self._slam_pid}")

            if self._bridge_pid is None:
                self._bridge_pid = pgrep_one(self.bridge_regex)
                if self._bridge_pid is not None:
                    self.get_logger().info(f"Found ros_gz_bridge pid={self._bridge_pid}")


        # Compute metrics
        scan_hz = self.scan_rate.rate_hz(t_now)
        map_hz = self.map_rate.rate_hz(t_now)
        scan_age = self._age_sec(self.last_scan_stamp)
        map_age = self._age_sec(self.last_map_stamp)

        cpu_slam = self._cpu_of_pid(self._slam_pid, label="slam_toolbox")
        cpu_bridge = self._cpu_of_pid(self._bridge_pid, label="ros_gz_bridge")

        cpu_hogs = self._cpu_total_hogs()

        sim_time = self._ros_time_sec()

        row = [
            t_rel,
            sim_time,
            scan_hz,
            map_hz,
            scan_age,
            map_age,
            cpu_slam,
            cpu_bridge,
            cpu_hogs,
            len(self.hogs),
        ]
        self.csv_w.writerow([f"{x:.6f}" if isinstance(x, float) else x for x in row])
        self.csv_f.flush()
        self.rows.append(row)

        if t_rel - self._last_debug_log_t >= self._debug_every_s:
            self._last_debug_log_t = t_rel
            self.get_logger().info(
                f"t={t_rel:.1f}s scan={scan_hz:.2f}Hz map={map_hz:.2f}Hz | "
                f"cpu slam={cpu_slam:.1f}% bridge={cpu_bridge:.1f}% hog_total={cpu_hogs:.1f}% "
                f"hogs={len(self.hogs)}"
            )

            if self._slam_pid is None:
                self.get_logger().warn("slam_toolbox PID not found yet (pgrep failed).")
            if self._bridge_pid is None:
                self.get_logger().warn("ros_gz_bridge PID not found yet (pgrep failed).")


        # Stop after duration
        if t_rel >= self.duration_s:
            self.get_logger().info("Duration reached. Stopping hogs and writing plot...")
            self._shutdown()

    def _shutdown(self):
        # terminate hogs
        for h in self.hogs:
            try:
                h.popen.send_signal(signal.SIGINT)
            except Exception:
                pass
        time.sleep(0.3)
        for h in self.hogs:
            try:
                h.popen.terminate()
            except Exception:
                pass

        try:
            self.csv_f.close()
        except Exception:
            pass

        # Plot
        try:
            self._plot()
            self.get_logger().info(f"Saved CSV: {self.out_csv}")
            self.get_logger().info(f"Saved plot: {self.out_png}")
        except Exception as e:
            self.get_logger().error(f"Plot failed: {e}")

        # end ROS
        rclpy.shutdown()

    def _plot(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        t = [r[0] for r in self.rows]
        scan = [r[2] for r in self.rows]
        mapp = [r[3] for r in self.rows]
        cpu_slam = [r[6] for r in self.rows]
        cpu_hogs = [r[8] for r in self.rows]

        fig = plt.figure(figsize=(12, 8))

        ax1 = fig.add_subplot(3, 1, 1)
        ax1.plot(t, scan, label="/scan rate (Hz)")
        ax1.set_ylabel("Hz")
        ax1.set_title("Topic rates + CPU load steps")
        ax1.grid(True)

        ax2 = fig.add_subplot(3, 1, 2, sharex=ax1)
        ax2.plot(t, mapp, label="/map rate (Hz)")
        ax2.set_ylabel("Hz")
        ax2.grid(True)

        ax3 = fig.add_subplot(3, 1, 3, sharex=ax1)
        ax3.plot(t, cpu_slam, label="slam_toolbox CPU%")
        ax3.plot(t, cpu_hogs, label="cpu_hog total CPU%")
        ax3.set_xlabel("time since start (s)")
        ax3.set_ylabel("CPU %")
        ax3.grid(True)

        # Red vertical lines for hog start events
        for (te, label) in self.events:
            for ax in (ax1, ax2, ax3):
                ax.axvline(te, color="red", linestyle="--", linewidth=1)
            ax1.text(te, max(scan) if scan else 0.0, label, rotation=90, va="bottom", fontsize=8)

        ax1.legend(loc="upper right")
        ax2.legend(loc="upper right")
        ax3.legend(loc="upper right")

        os.makedirs(os.path.dirname(self.out_png) or ".", exist_ok=True)
        fig.tight_layout()
        fig.savefig(self.out_png, dpi=160)
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Measure /scan & /map rates while adding cpu_hog processes over time.")
    ap.add_argument("--loads", required=True, help="Comma-separated loads for each new cpu_hog, e.g. 0.6,0.8,0.9")
    ap.add_argument("--start-after", type=float, default=10.0, help="Seconds to wait before starting first hog")
    ap.add_argument("--step", type=float, default=20.0, help="Seconds between starting additional hogs")
    ap.add_argument("--duration", type=float, default=120.0, help="Total experiment duration (s)")
    ap.add_argument("--log-period", type=float, default=0.2, help="Logging period (s)")
    ap.add_argument("--window", type=float, default=5.0, help="Rate estimation window (s)")
    ap.add_argument("--outdir", default="metrics_out", help="Output directory")
    ap.add_argument("--prefix", default=None, help="Output file prefix (default: timestamp)")
    ap.add_argument("--slam-regex", default=r"slam_toolbox", help="Regex to find slam_toolbox process in `ps`")
    ap.add_argument("--bridge-regex", default=r"ros_gz_bridge", help="Regex to find ros_gz_bridge process in `ps`")
    ap.add_argument("--dry-run", action="store_true", help="Do not start cpu_hog, only record events")
    args = ap.parse_args()

    loads = parse_loads(args.loads)

    ts = time.strftime("%Y%m%d_%H%M%S")
    prefix = args.prefix or ts
    out_csv = os.path.join(args.outdir, f"{prefix}.csv")
    out_png = os.path.join(args.outdir, f"{prefix}.png")

    rclpy.init()
    node = ExperimentNode(
        loads=loads,
        start_after_s=args.start_after,
        step_s=args.step,
        duration_s=args.duration,
        log_period_s=args.log_period,
        window_s=args.window,
        out_csv=out_csv,
        out_png=out_png,
        slam_regex=args.slam_regex,
        bridge_regex=args.bridge_regex,
        dry_run=args.dry_run,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
