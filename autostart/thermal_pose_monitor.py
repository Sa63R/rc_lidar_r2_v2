#!/usr/bin/env python3
"""Record temperature, CPU load, and pose stability for thermal diagnosis."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import statistics
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Deque, Iterable

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


CSV_FIELDS = [
    "time_iso",
    "elapsed_sec",
    "max_temp_c",
    "load1",
    "load5",
    "load15",
    "cpu_freq_min_mhz",
    "cpu_freq_max_mhz",
    "cpu_freq_avg_mhz",
    "raw_x",
    "raw_y",
    "raw_z",
    "filtered_x",
    "filtered_y",
    "filtered_z",
    "raw_filtered_delta_xy_m",
    "raw_window_x_p2p_m",
    "raw_window_y_p2p_m",
    "raw_window_xy_p2p_m",
    "filtered_window_x_p2p_m",
    "filtered_window_y_p2p_m",
    "filtered_window_xy_p2p_m",
    "map_quality_accepted",
    "map_quality_converged",
    "map_quality_fitness",
    "map_quality_jump_xy_m",
    "map_quality_scan_points",
    "map_quality_map_points",
    "raw_age_sec",
    "filtered_age_sec",
    "quality_age_sec",
]


def finite(value: float | None) -> float:
    if value is None:
        return math.nan
    return float(value)


def read_loadavg() -> tuple[float, float, float]:
    try:
        fields = Path("/proc/loadavg").read_text(encoding="utf-8").split()
        return float(fields[0]), float(fields[1]), float(fields[2])
    except Exception:
        return math.nan, math.nan, math.nan


def read_max_temperature_c() -> float:
    values: list[float] = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            raw = float(path.read_text(encoding="utf-8").strip())
        except Exception:
            continue
        if raw > 1000.0:
            raw /= 1000.0
        values.append(raw)
    return max(values) if values else math.nan


def read_cpu_freq_mhz() -> tuple[float, float, float]:
    values: list[float] = []
    for path in Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq"):
        try:
            raw = float(path.read_text(encoding="utf-8").strip())
        except Exception:
            continue
        values.append(raw / 1000.0)
    if not values:
        return math.nan, math.nan, math.nan
    return min(values), max(values), sum(values) / len(values)


def window_p2p(samples: Iterable[tuple[float, float, float]]) -> tuple[float, float, float]:
    xs = [x for _, x, _ in samples if math.isfinite(x)]
    ys = [y for _, _, y in samples if math.isfinite(y)]
    if not xs or not ys:
        return math.nan, math.nan, math.nan
    x_p2p = max(xs) - min(xs)
    y_p2p = max(ys) - min(ys)
    xy_p2p = math.hypot(x_p2p, y_p2p)
    return x_p2p, y_p2p, xy_p2p


def pearson(xs: list[float], ys: list[float]) -> float:
    pairs = [
        (x, y)
        for x, y in zip(xs, ys)
        if math.isfinite(x) and math.isfinite(y)
    ]
    if len(pairs) < 3:
        return math.nan
    x_vals = [p[0] for p in pairs]
    y_vals = [p[1] for p in pairs]
    x_mean = statistics.fmean(x_vals)
    y_mean = statistics.fmean(y_vals)
    numerator = sum((x - x_mean) * (y - y_mean) for x, y in pairs)
    x_den = math.sqrt(sum((x - x_mean) ** 2 for x in x_vals))
    y_den = math.sqrt(sum((y - y_mean) ** 2 for y in y_vals))
    if x_den == 0.0 or y_den == 0.0:
        return math.nan
    return numerator / (x_den * y_den)


def series_stats(rows: list[dict[str, float]], key: str) -> dict[str, float]:
    values = [float(row[key]) for row in rows if math.isfinite(float(row.get(key, math.nan)))]
    if not values:
        return {"min": math.nan, "max": math.nan, "mean": math.nan}
    return {
        "min": min(values),
        "max": max(values),
        "mean": statistics.fmean(values),
    }


def json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


class ThermalPoseMonitor(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("thermal_pose_monitor")
        self.args = args
        self.start_time = time.monotonic()
        self.stop_requested = False
        self.rows: list[dict[str, float | str]] = []

        self.raw_pose: tuple[float, float, float, float] | None = None
        self.filtered_pose: tuple[float, float, float, float] | None = None
        self.quality: tuple[float, list[float]] | None = None

        self.raw_window: Deque[tuple[float, float, float]] = deque()
        self.filtered_window: Deque[tuple[float, float, float]] = deque()

        self.output_dir = Path(args.output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = Path(args.csv_path or self.output_dir / "thermal_pose_monitor.csv")
        self.summary_path = Path(
            args.summary_path or self.output_dir / "thermal_pose_monitor_summary.json"
        )

        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()

        self.create_subscription(PoseStamped, args.raw_topic, self.on_raw_pose, 20)
        self.create_subscription(PoseStamped, args.filtered_topic, self.on_filtered_pose, 20)
        self.create_subscription(Float32MultiArray, args.quality_topic, self.on_quality, 20)
        self.timer = self.create_timer(args.interval, self.sample)

        self.get_logger().info(
            f"thermal pose monitor logging csv={self.csv_path} "
            f"summary={self.summary_path} raw={args.raw_topic} "
            f"filtered={args.filtered_topic} quality={args.quality_topic}"
        )

    def on_raw_pose(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        p = msg.pose.position
        self.raw_pose = (now, float(p.x), float(p.y), float(p.z))
        self.raw_window.append((now, float(p.x), float(p.y)))
        self.trim_window(self.raw_window, now)

    def on_filtered_pose(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        p = msg.pose.position
        self.filtered_pose = (now, float(p.x), float(p.y), float(p.z))
        self.filtered_window.append((now, float(p.x), float(p.y)))
        self.trim_window(self.filtered_window, now)

    def on_quality(self, msg: Float32MultiArray) -> None:
        self.quality = (time.monotonic(), [float(value) for value in msg.data])

    def trim_window(self, window: Deque[tuple[float, float, float]], now: float) -> None:
        cutoff = now - self.args.window
        while window and window[0][0] < cutoff:
            window.popleft()

    def sample(self) -> None:
        now = time.monotonic()
        self.trim_window(self.raw_window, now)
        self.trim_window(self.filtered_window, now)

        temp_c = read_max_temperature_c()
        load1, load5, load15 = read_loadavg()
        freq_min, freq_max, freq_avg = read_cpu_freq_mhz()

        raw_x = raw_y = raw_z = math.nan
        raw_age = math.nan
        if self.raw_pose:
            stamp, raw_x, raw_y, raw_z = self.raw_pose
            raw_age = now - stamp

        filtered_x = filtered_y = filtered_z = math.nan
        filtered_age = math.nan
        if self.filtered_pose:
            stamp, filtered_x, filtered_y, filtered_z = self.filtered_pose
            filtered_age = now - stamp

        delta_xy = math.nan
        if all(math.isfinite(value) for value in (raw_x, raw_y, filtered_x, filtered_y)):
            delta_xy = math.hypot(raw_x - filtered_x, raw_y - filtered_y)

        raw_x_p2p, raw_y_p2p, raw_xy_p2p = window_p2p(self.raw_window)
        filtered_x_p2p, filtered_y_p2p, filtered_xy_p2p = window_p2p(self.filtered_window)

        accepted = converged = fitness = quality_jump = scan_points = map_points = math.nan
        quality_age = math.nan
        if self.quality:
            stamp, values = self.quality
            quality_age = now - stamp
            accepted = values[0] if len(values) > 0 else math.nan
            converged = values[1] if len(values) > 1 else math.nan
            fitness = values[2] if len(values) > 2 else math.nan
            quality_jump = values[7] if len(values) > 7 else math.nan
            scan_points = values[5] if len(values) > 5 else math.nan
            map_points = values[6] if len(values) > 6 else math.nan

        row = {
            "time_iso": datetime.now().isoformat(timespec="seconds"),
            "elapsed_sec": now - self.start_time,
            "max_temp_c": temp_c,
            "load1": load1,
            "load5": load5,
            "load15": load15,
            "cpu_freq_min_mhz": freq_min,
            "cpu_freq_max_mhz": freq_max,
            "cpu_freq_avg_mhz": freq_avg,
            "raw_x": raw_x,
            "raw_y": raw_y,
            "raw_z": raw_z,
            "filtered_x": filtered_x,
            "filtered_y": filtered_y,
            "filtered_z": filtered_z,
            "raw_filtered_delta_xy_m": delta_xy,
            "raw_window_x_p2p_m": raw_x_p2p,
            "raw_window_y_p2p_m": raw_y_p2p,
            "raw_window_xy_p2p_m": raw_xy_p2p,
            "filtered_window_x_p2p_m": filtered_x_p2p,
            "filtered_window_y_p2p_m": filtered_y_p2p,
            "filtered_window_xy_p2p_m": filtered_xy_p2p,
            "map_quality_accepted": accepted,
            "map_quality_converged": converged,
            "map_quality_fitness": fitness,
            "map_quality_jump_xy_m": quality_jump,
            "map_quality_scan_points": scan_points,
            "map_quality_map_points": map_points,
            "raw_age_sec": raw_age,
            "filtered_age_sec": filtered_age,
            "quality_age_sec": quality_age,
        }

        self.rows.append(row)
        self.writer.writerow(row)
        self.csv_file.flush()

        if (
            self.args.summary_interval > 0.0
            and len(self.rows) > 1
            and (now - self.start_time) % self.args.summary_interval < self.args.interval
        ):
            self.write_summary()

    def write_summary(self) -> None:
        numeric_rows = [
            {key: float(value) for key, value in row.items() if key != "time_iso"}
            for row in self.rows
        ]
        if not numeric_rows:
            summary = {"sample_count": 0}
        else:
            temps = [row["max_temp_c"] for row in numeric_rows]
            loads = [row["load1"] for row in numeric_rows]
            raw_p2p = [row["raw_window_xy_p2p_m"] for row in numeric_rows]
            filtered_p2p = [row["filtered_window_xy_p2p_m"] for row in numeric_rows]
            quality_jump = [row["map_quality_jump_xy_m"] for row in numeric_rows]
            summary = {
                "sample_count": len(numeric_rows),
                "duration_sec": numeric_rows[-1]["elapsed_sec"],
                "csv_path": str(self.csv_path),
                "temperature_c": series_stats(numeric_rows, "max_temp_c"),
                "load1": series_stats(numeric_rows, "load1"),
                "load5": series_stats(numeric_rows, "load5"),
                "cpu_freq_avg_mhz": series_stats(numeric_rows, "cpu_freq_avg_mhz"),
                "raw_window_xy_p2p_m": series_stats(numeric_rows, "raw_window_xy_p2p_m"),
                "filtered_window_xy_p2p_m": series_stats(
                    numeric_rows, "filtered_window_xy_p2p_m"
                ),
                "raw_filtered_delta_xy_m": series_stats(
                    numeric_rows, "raw_filtered_delta_xy_m"
                ),
                "map_quality_fitness": series_stats(numeric_rows, "map_quality_fitness"),
                "map_quality_jump_xy_m": series_stats(numeric_rows, "map_quality_jump_xy_m"),
                "correlation": {
                    "temp_vs_raw_window_xy_p2p": pearson(temps, raw_p2p),
                    "temp_vs_filtered_window_xy_p2p": pearson(temps, filtered_p2p),
                    "temp_vs_map_quality_jump_xy": pearson(temps, quality_jump),
                    "load1_vs_raw_window_xy_p2p": pearson(loads, raw_p2p),
                    "load1_vs_filtered_window_xy_p2p": pearson(loads, filtered_p2p),
                    "load1_vs_map_quality_jump_xy": pearson(loads, quality_jump),
                },
            }

        self.summary_path.write_text(
            json.dumps(json_safe(summary), indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8",
        )

    def close(self) -> None:
        try:
            self.write_summary()
        finally:
            self.csv_file.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record temperature/load/frequency and robot pose jitter."
    )
    parser.add_argument("--output-dir", default=os.environ.get("LOG_DIR", "."))
    parser.add_argument("--csv-path", default="")
    parser.add_argument("--summary-path", default="")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--window", type=float, default=10.0)
    parser.add_argument("--summary-interval", type=float, default=30.0)
    parser.add_argument("--raw-topic", default="/robot/pose")
    parser.add_argument("--filtered-topic", default="/robot/pose_filtered")
    parser.add_argument("--quality-topic", default="/map_lidar/quality")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ThermalPoseMonitor(args)

    stop = {"requested": False}

    def request_stop(signum, _frame):
        stop["requested"] = True
        node.get_logger().info(f"signal {signum} received; writing summary and exiting")

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    try:
        while rclpy.ok() and not stop["requested"]:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
