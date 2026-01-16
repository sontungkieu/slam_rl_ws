#!/usr/bin/env python3
import argparse
import csv
import math
import os
import re
import json
import signal
import subprocess
import time
import atexit
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
from std_msgs.msg import String

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

def pick_best_pid_for_node(node_name: str, timeout_s: float = 3.0) -> Optional[int]:
    """
    Wait up to timeout_s for the REAL process (installed cpu_hog) to appear.
    Only if timeout -> fallback to wrapper PID.
    """
    deadline = time.time() + timeout_s
    pat = f"__node:={node_name}"
    wrapper_pid = None

    while time.time() < deadline:
        try:
            lines = subprocess.check_output(["pgrep", "-fa", pat], text=True).splitlines()
        except subprocess.CalledProcessError:
            lines = []

        found_real = None
        wrapper_pid = None

        for ln in lines:
            try:
                pid = int(ln.split(None, 1)[0])
                cmd = ln.split(None, 1)[1] if " " in ln else ""
            except Exception:
                continue

            # ưu tiên PID thật
            if "/install/rl_resource_manager/lib/rl_resource_manager/cpu_hog" in cmd:
                found_real = pid
                break

            # nhớ wrapper để fallback
            if " ros2 run rl_resource_manager cpu_hog " in cmd or "/opt/ros/" in cmd and "ros2 run" in cmd:
                wrapper_pid = pid

        if found_real is not None:
            return found_real

        # thấy wrapper nhưng chưa thấy real -> chờ thêm, đừng return vội
        time.sleep(0.05)

    return wrapper_pid


@dataclass
class HogProc:
    popen: subprocess.Popen
    load: float
    start_t: float
    name: str
    cpu_pid: int  # PID process thật (child), không phải wrapper ros2


class HzStatsTracker:
    def __init__(self, window_s: float):
        self.window_s = window_s
        self.last_t: Optional[float] = None
        self.samples: Deque[Tuple[float, float]] = deque()  # (t_now, dt)

    def add(self, t_now: float):
        if self.last_t is not None:
            dt = t_now - self.last_t
            if dt > 0:
                self.samples.append((t_now, dt))
        self.last_t = t_now
        self._prune(t_now)

    def _prune(self, t_now: float):
        cutoff = t_now - self.window_s
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def stats(self, t_now: float) -> Tuple[float, float, float, float, int]:
        self._prune(t_now)
        if not self.samples:
            return 0.0, float("nan"), float("nan"), float("nan"), 0
        dts = [dt for _, dt in self.samples]
        mean_dt = sum(dts) / len(dts)
        rate_hz = (1.0 / mean_dt) if mean_dt > 1e-9 else 0.0
        mn = min(dts)
        mx = max(dts)
        var = sum((dt - mean_dt) ** 2 for dt in dts) / len(dts)
        std = math.sqrt(var)
        return rate_hz, mn, mx, std, len(dts)


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
        self.slam_regex = "async_slam_toolbox_node"
        print(self.slam_regex)
        self.bridge_regex = bridge_regex
        self.dry_run = dry_run
        # ---- CPU sampling cache: pid -> (last_wall_mono, last_cpu_seconds) ----
        self._cpu_last = {}

        # Subscriptions
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.sub_map = self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.sub_hog_status = self.create_subscription(String, "/cpu_hog_status", self.on_hog_status, 10)

        # Track rates based on arrival times
        self.scan_rate = HzStatsTracker(window_s)
        self.map_rate  = HzStatsTracker(window_s)


        self.last_scan_stamp = None
        self.last_map_stamp = None

        self.t0 = now_mono()
        # Unique id per run so we can clean only hogs created by this run
        self.run_id = str(os.getpid())

        # Ensure cleanup even if user Ctrl+C or plot crashes
        atexit.register(self._force_cleanup_all_hogs, "atexit")
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)
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
            "scan_dt_min_s",
            "scan_dt_max_s",
            "scan_dt_std_s",
            "scan_window_n",

            "map_hz",
            "map_dt_min_s",
            "map_dt_max_s",
            "map_dt_std_s",
            "map_window_n",

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
    
    def _sig_handler(self, signum, frame):
        self.get_logger().warn(f"Received signal {signum}, cleaning up hogs...")
        self._force_cleanup_all_hogs(f"signal {signum}")
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def on_hog_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            name = data.get("name")
            pid = data.get("pid")
            if name and pid:
                # Update PID in self.hogs
                for h in self.hogs:
                    if h.name == name:
                        # Found the hog
                        if getattr(h, "cpu_pid", None) != pid:
                            self.get_logger().info(f"Updated PID for {name}: {h.cpu_pid} -> {pid} (via topic)")
                            h.cpu_pid = pid
                            # Reset proc cache for this PID so we pick up new process
                            self._proc_cache.pop(pid, None)
                        break
        except Exception:
            pass


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
    
    def _force_cleanup_all_hogs(self, reason: str):
        # Kill only hogs created by THIS run_id (by node name prefix), plus any popens we tracked.
        self.get_logger().warn(f"Cleaning up hogs ({reason})... count={len(self.hogs)}")

        # 1) kill what we spawned (robust)
        for h in list(self.hogs):
            try:
                self._kill_popen_group(h.popen, h.name, timeout_s=2.0)
            except Exception:
                pass

        # 2) also pkill by unique node name prefix (in case popen list got lost)
        try:
            subprocess.call(["pkill", "-f", f"__node:=cpu_hog_{self.run_id}_"])
        except Exception:
            pass

        # prune list
        alive = []
        for h in self.hogs:
            if h.popen.poll() is None:
                alive.append(h)
        self.hogs = alive

    def _start_hog(self, load: float, idx: int):
        name = f"cpu_hog_{self.run_id}_{idx}_{int(load*100):02d}"
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

        logdir = os.path.join(os.path.dirname(self.out_csv) or ".", "hog_logs")
        os.makedirs(logdir, exist_ok=True)
        stdout_f = open(os.path.join(logdir, f"{name}.out"), "w")
        stderr_f = open(os.path.join(logdir, f"{name}.err"), "w")

        # start new process group so killpg works
        p = subprocess.Popen(
            cmd,
            stdout=stdout_f,
            stderr=stderr_f,
            start_new_session=True,
        )

        # IMPORTANT: p.pid is wrapper "ros2 run", we want the real child hog PID
        child = pick_best_pid_for_node(name, timeout_s=5.0)
        cpu_pid = child if child is not None else p.pid

        self.hogs.append(HogProc(popen=p, load=load, start_t=now_mono(), name=name, cpu_pid=cpu_pid))
        self.events.append((now_mono() - self.t0, f"hog {idx} load={load}"))

        self.get_logger().info(f"Started {name} parent(pid)={p.pid} cpu_pid={cpu_pid} load={load}")
        self.get_logger().info(f"CPU monitor attached: {name} pid={cpu_pid}")

    
    def _kill_popen_group(self, p: subprocess.Popen, label: str, timeout_s: float = 2.0):
        if p is None:
            return

        if p.poll() is not None:
            return  # already exited

        try:
            pgid = os.getpgid(p.pid)
        except Exception:
            pgid = None

        # Try graceful stop
        try:
            if pgid is not None:
                os.killpg(pgid, signal.SIGINT)
            else:
                p.send_signal(signal.SIGINT)
        except Exception:
            pass

        # Wait a bit
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if p.poll() is not None:
                return
            time.sleep(0.05)

        # Escalate to SIGTERM
        try:
            if pgid is not None:
                os.killpg(pgid, signal.SIGTERM)
            else:
                p.terminate()
        except Exception:
            pass

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if p.poll() is not None:
                return
            time.sleep(0.05)

        # Final: SIGKILL
        try:
            if pgid is not None:
                os.killpg(pgid, signal.SIGKILL)
            else:
                p.kill()
        except Exception:
            pass


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
            pid = getattr(h, "cpu_pid", h.popen.pid)  # fallback nếu chưa có cpu_pid
            v = self._cpu_of_pid(pid, label=h.name)
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
        scan_hz, scan_dt_min, scan_dt_max, scan_dt_std, scan_n = self.scan_rate.stats(t_now)
        map_hz,  map_dt_min,  map_dt_max,  map_dt_std,  map_n  = self.map_rate.stats(t_now)
        scan_age = self._age_sec(self.last_scan_stamp)
        map_age = self._age_sec(self.last_map_stamp)

        cpu_slam = self._cpu_of_pid(self._slam_pid, label="slam_toolbox")
        cpu_bridge = self._cpu_of_pid(self._bridge_pid, label="ros_gz_bridge")

        # IMPORTANT: cpu_hogs must sum REAL hog child pids, not ros2 wrapper pid
        cpu_hogs = self._cpu_total_hogs()

        sim_time = self._ros_time_sec()

        row = [
            t_rel,
            sim_time,

            scan_hz,
            scan_dt_min,
            scan_dt_max,
            scan_dt_std,
            float(scan_n),

            map_hz,
            map_dt_min,
            map_dt_max,
            map_dt_std,
            float(map_n),

            scan_age,
            map_age,

            cpu_slam,
            cpu_bridge,
            cpu_hogs,
            float(len(self.hogs)),
        ]

        self.csv_w.writerow([f"{x:.6f}" if isinstance(x, float) else x for x in row])
        self.csv_f.flush()
        self.rows.append(row)

        # Single debug block (remove duplicate)
        if t_rel - self._last_debug_log_t >= self._debug_every_s:
            self._last_debug_log_t = t_rel
            self.get_logger().info(
                f"t={t_rel:.1f}s scan={scan_hz:.2f}Hz map={map_hz:.2f}Hz | "
                f"scan dt[min={scan_dt_min:.3f} max={scan_dt_max:.3f} std={scan_dt_std:.3f}] n={scan_n} | "
                f"map  dt[min={map_dt_min:.3f} max={map_dt_max:.3f} std={map_dt_std:.3f}] n={map_n} | "
                f"cpu slam={cpu_slam:.1f}% bridge={cpu_bridge:.1f}% hog_total={cpu_hogs:.1f}% hogs={len(self.hogs)}"
            )

            if self._slam_pid is None:
                self.get_logger().warn("slam_toolbox PID not found yet (pgrep failed).")
            if self._bridge_pid is None:
                self.get_logger().warn("ros_gz_bridge PID not found yet (pgrep failed).")

            if self.hogs:
                self.get_logger().info(
                    "hog pids: " + ", ".join(f"{h.name}:{getattr(h,'cpu_pid',h.popen.pid)}" for h in self.hogs)
                )

        # Stop after duration
        if t_rel >= self.duration_s:
            self.get_logger().info("Duration reached. Stopping hogs and writing plot...")
            self._shutdown()


    def _shutdown(self):
        # 1) Stop/kill all hogs robustly (kill whole process groups)
        try:
            self._force_cleanup_all_hogs("normal shutdown")
        except Exception as e:
            self.get_logger().warn(f"Cleanup hogs failed: {e}")

        # 2) Close CSV
        try:
            self.csv_f.close()
        except Exception:
            pass

        # 3) Plot
        try:
            self._plot()
            self.get_logger().info(f"Saved CSV: {self.out_csv}")
            self.get_logger().info(f"Saved plot: {self.out_png}")
        except Exception as e:
            self.get_logger().error(f"Plot failed: {e}")

        # 4) End ROS
        try:
            rclpy.shutdown()
        except Exception:
            pass


    def _plot(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import csv
        import os
        import math

        # --- Read CSV back by column names (robust to column order changes) ---
        t = []
        scan_hz = []
        map_hz = []
        scan_dt_std = []
        map_dt_std = []
        cpu_slam = []
        cpu_hogs = []

        with open(self.out_csv, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for r in reader:
                def g(key, default=float("nan")):
                    try:
                        return float(r.get(key, default))
                    except Exception:
                        return default

                t.append(g("t_rel_s"))
                scan_hz.append(g("scan_hz"))
                map_hz.append(g("map_hz"))
                scan_dt_std.append(g("scan_dt_std_s"))
                map_dt_std.append(g("map_dt_std_s"))
                cpu_slam.append(g("cpu_slam_percent"))
                cpu_hogs.append(g("cpu_hog_total_percent"))

        fig = plt.figure(figsize=(12, 10))

        ax1 = fig.add_subplot(4, 1, 1)
        ax1.plot(t, scan_hz, label="/scan rate (Hz)")
        ax1.set_ylabel("Hz")
        ax1.set_title("Topic rates + jitter + CPU load steps")
        ax1.grid(True)

        ax2 = fig.add_subplot(4, 1, 2, sharex=ax1)
        ax2.plot(t, map_hz, label="/map rate (Hz)")
        ax2.set_ylabel("Hz")
        ax2.grid(True)

        ax3 = fig.add_subplot(4, 1, 3, sharex=ax1)
        ax3.plot(t, scan_dt_std, label="/scan dt std (s)")
        ax3.plot(t, map_dt_std, label="/map dt std (s)")
        ax3.set_ylabel("std of dt (s)")
        ax3.grid(True)

        ax4 = fig.add_subplot(4, 1, 4, sharex=ax1)
        ax4.plot(t, cpu_slam, label="slam_toolbox CPU%")
        ax4.plot(t, cpu_hogs, label="cpu_hog total CPU%")
        ax4.set_xlabel("time since start (s)")
        ax4.set_ylabel("CPU %")
        ax4.grid(True)

        # Red vertical lines for hog start events
        for (te, label) in self.events:
            for ax in (ax1, ax2, ax3, ax4):
                ax.axvline(te, color="red", linestyle="--", linewidth=1)
            # annotate only on top subplot
            y = max([v for v in scan_hz if math.isfinite(v)] + [0.0])
            ax1.text(te, y, label, rotation=90, va="bottom", fontsize=8)

        ax1.legend(loc="upper right")
        ax2.legend(loc="upper right")
        ax3.legend(loc="upper right")
        ax4.legend(loc="upper right")

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
