#!/usr/bin/env python3
"""Record temperature, CPU load, and robot pose stability at low rate."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import socket
import statistics
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, Iterable

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


PoseSample = tuple[float, float, float, float, float]


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
    "raw_yaw_deg",
    "continuous_x",
    "continuous_y",
    "continuous_z",
    "continuous_yaw_deg",
    "filtered_x",
    "filtered_y",
    "filtered_z",
    "filtered_yaw_deg",
    "beta_x",
    "beta_y",
    "beta_z",
    "beta_yaw_deg",
    "stable_x",
    "stable_y",
    "stable_z",
    "stable_yaw_deg",
    "raw_filtered_delta_xy_m",
    "raw_filtered_delta_yaw_deg",
    "raw_continuous_delta_xy_m",
    "continuous_filtered_delta_xy_m",
    "raw_beta_delta_xy_m",
    "filtered_beta_delta_xy_m",
    "raw_stable_delta_xy_m",
    "filtered_stable_delta_xy_m",
    "raw_step_xy_m",
    "raw_speed_xy_mps",
    "raw_yaw_rate_degps",
    "continuous_step_xy_m",
    "continuous_speed_xy_mps",
    "continuous_yaw_rate_degps",
    "filtered_step_xy_m",
    "filtered_speed_xy_mps",
    "filtered_yaw_rate_degps",
    "beta_step_xy_m",
    "beta_speed_xy_mps",
    "beta_yaw_rate_degps",
    "stable_step_xy_m",
    "stable_speed_xy_mps",
    "stable_yaw_rate_degps",
    "motion_factor",
    "is_stationary",
    "raw_window_x_p2p_m",
    "raw_window_y_p2p_m",
    "raw_window_xy_p2p_m",
    "filtered_window_x_p2p_m",
    "filtered_window_y_p2p_m",
    "filtered_window_xy_p2p_m",
    "beta_window_x_p2p_m",
    "beta_window_y_p2p_m",
    "beta_window_xy_p2p_m",
    "stable_window_x_p2p_m",
    "stable_window_y_p2p_m",
    "stable_window_xy_p2p_m",
    "stationary_raw_window_x_p2p_m",
    "stationary_raw_window_y_p2p_m",
    "stationary_raw_window_xy_p2p_m",
    "stationary_filtered_window_x_p2p_m",
    "stationary_filtered_window_y_p2p_m",
    "stationary_filtered_window_xy_p2p_m",
    "spike_candidate",
    "sustained_motion",
    "static_context_raw_xy_p2p_m",
    "static_context_filtered_xy_p2p_m",
    "map_quality_accepted",
    "map_quality_converged",
    "map_quality_fitness",
    "map_quality_jump_xy_m",
    "map_quality_scan_points",
    "map_quality_map_points",
    "map_quality_reason_code",
    "map_quality_align_ms",
    "map_quality_raw_scan_points",
    "map_quality_scan_age_sec",
    "map_quality_odom_age_sec",
    "map_quality_output_jump_yaw_deg",
    "map_quality_map_to_odom_delta_xy_m",
    "map_quality_map_to_odom_delta_yaw_deg",
    "map_quality_overlap_ratio",
    "map_quality_scan_odom_delta_sec",
    "map_quality_scan_reused",
    "map_quality_odom_buffer_size",
    "map_quality_localize_busy",
    "map_quality_preprocess_ms",
    "map_quality_total_ms",
    "filter_status_accepted",
    "filter_status_raw_step_xy_m",
    "filter_status_speed_xy_mps",
    "filter_status_alpha",
    "filter_status_reject_count",
    "filter_status_consecutive_rejects",
    "filter_status_recovered",
    "filter_status_motion_factor",
    "filter_status_yaw_rate_degps",
    "filter_status_reason_code",
    "filter_status_hold_state_code",
    "filter_status_confirm_count",
    "beta_filter_status_accepted",
    "beta_filter_status_raw_step_xy_m",
    "beta_filter_status_speed_xy_mps",
    "beta_filter_status_alpha",
    "beta_filter_status_reject_count",
    "beta_filter_status_consecutive_rejects",
    "beta_filter_status_recovered",
    "beta_filter_status_motion_factor",
    "beta_filter_status_yaw_rate_degps",
    "beta_filter_status_reason_code",
    "beta_filter_status_hold_state_code",
    "beta_filter_status_confirm_count",
    "stable_filter_status_accepted",
    "stable_filter_status_raw_step_xy_m",
    "stable_filter_status_speed_xy_mps",
    "stable_filter_status_alpha",
    "stable_filter_status_reject_count",
    "stable_filter_status_consecutive_rejects",
    "stable_filter_status_recovered",
    "stable_filter_status_motion_factor",
    "stable_filter_status_yaw_rate_degps",
    "stable_filter_status_reason_code",
    "stable_filter_status_hold_state_code",
    "stable_filter_status_confirm_count",
    "continuous_status_ready",
    "continuous_status_map_update_accepted",
    "continuous_status_correction_alpha",
    "continuous_status_correction_age_sec",
    "continuous_status_odom_age_sec",
    "continuous_status_stamp_delta_sec",
    "continuous_status_candidate_delta_xy_m",
    "continuous_status_candidate_delta_yaw_deg",
    "continuous_status_quality_fitness",
    "continuous_status_update_count",
    "continuous_status_rejected_count",
    "continuous_status_match_mode_code",
    "continuous_status_reason_code",
    "yaw_fusion_enabled",
    "yaw_fusion_ready",
    "yaw_fusion_source_code",
    "yaw_fusion_fast_age_sec",
    "yaw_fusion_map_age_sec",
    "yaw_fusion_map_ref_yaw_deg",
    "yaw_fusion_fast_yaw_deg",
    "yaw_fusion_fused_yaw_deg",
    "yaw_fusion_delta_yaw_deg",
    "yaw_fusion_map_correction_alpha",
    "raw_age_sec",
    "continuous_age_sec",
    "filtered_age_sec",
    "beta_age_sec",
    "stable_age_sec",
    "quality_age_sec",
    "filter_status_age_sec",
    "beta_filter_status_age_sec",
    "stable_filter_status_age_sec",
    "continuous_status_age_sec",
    "yaw_fusion_status_age_sec",
]


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


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle_rad(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def ramp(value: float, low: float, high: float) -> float:
    if not math.isfinite(value):
        return math.nan
    if high <= low:
        return 1.0 if value >= high else 0.0
    return min(1.0, max(0.0, (value - low) / (high - low)))


def window_p2p(samples: Iterable[PoseSample]) -> tuple[float, float, float]:
    xs = [sample[1] for sample in samples if math.isfinite(sample[1])]
    ys = [sample[2] for sample in samples if math.isfinite(sample[2])]
    if not xs or not ys:
        return math.nan, math.nan, math.nan
    x_p2p = max(xs) - min(xs)
    y_p2p = max(ys) - min(ys)
    xy_p2p = math.hypot(x_p2p, y_p2p)
    return x_p2p, y_p2p, xy_p2p


def pose_step(
    previous: PoseSample | None,
    current: PoseSample | None,
) -> tuple[float, float, float]:
    if previous is None or current is None:
        return math.nan, math.nan, math.nan
    dt = current[0] - previous[0]
    if dt < 1e-6:
        return 0.0, 0.0, 0.0
    xy_step = math.hypot(current[1] - previous[1], current[2] - previous[2])
    yaw_step = abs(shortest_angle_rad(current[4] - previous[4]))
    return xy_step, xy_step / dt, math.degrees(yaw_step / dt)


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
    values = [row[key] for row in rows if math.isfinite(row.get(key, math.nan))]
    if not values:
        return {"min": math.nan, "max": math.nan, "mean": math.nan}
    return {"min": min(values), "max": max(values), "mean": statistics.fmean(values)}


def json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


def fsync_directory(path: Path) -> None:
    try:
        fd = os.open(str(path), os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    with tmp_path.open("w", encoding="utf-8") as handle:
        json.dump(json_safe(payload), handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp_path, path)
    fsync_directory(path.parent)


class ThermalPoseMonitor(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("thermal_pose_monitor")
        self.args = args
        self.start_time = time.monotonic()
        self.rows: list[dict[str, float | str]] = []
        self.last_summary_elapsed = 0.0
        self.last_csv_fsync = self.start_time

        self.raw_pose: PoseSample | None = None
        self.continuous_pose: PoseSample | None = None
        self.filtered_pose: PoseSample | None = None
        self.beta_pose: PoseSample | None = None
        self.stable_pose: PoseSample | None = None
        self.quality: tuple[float, list[float]] | None = None
        self.filter_status: tuple[float, list[float]] | None = None
        self.beta_filter_status: tuple[float, list[float]] | None = None
        self.stable_filter_status: tuple[float, list[float]] | None = None
        self.continuous_status: tuple[float, list[float]] | None = None
        self.yaw_fusion_status: tuple[float, list[float]] | None = None
        self.previous_raw_pose: PoseSample | None = None
        self.previous_continuous_pose: PoseSample | None = None
        self.previous_filtered_pose: PoseSample | None = None
        self.previous_beta_pose: PoseSample | None = None
        self.previous_stable_pose: PoseSample | None = None
        self.callback_previous_raw_pose: PoseSample | None = None
        self.callback_previous_continuous_pose: PoseSample | None = None
        self.callback_previous_filtered_pose: PoseSample | None = None
        self.callback_previous_beta_pose: PoseSample | None = None
        self.callback_previous_stable_pose: PoseSample | None = None

        self.raw_window: Deque[PoseSample] = deque()
        self.continuous_window: Deque[PoseSample] = deque()
        self.filtered_window: Deque[PoseSample] = deque()
        self.beta_window: Deque[PoseSample] = deque()
        self.stable_window: Deque[PoseSample] = deque()
        self.stationary_raw_window: Deque[PoseSample] = deque()
        self.stationary_filtered_window: Deque[PoseSample] = deque()
        self.static_context_raw_window: Deque[PoseSample] = deque()
        self.static_context_filtered_window: Deque[PoseSample] = deque()
        self.event_buffer: Deque[dict[str, float | str]] = deque()
        self.open_events: list[dict[str, Any]] = []
        self.event_count = 0
        self.last_event_by_reason: dict[str, float] = {}
        self.motion_started_at: float | None = None

        self.output_dir = Path(args.output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = Path(args.csv_path or self.output_dir / "health_monitor.csv")
        self.summary_path = Path(args.summary_path or self.output_dir / "health_monitor_summary.json")
        self.event_path = Path(args.event_path or self.output_dir / "health_events.jsonl")
        self.event_context_path = Path(
            args.event_context_path or self.output_dir / "health_event_context.csv"
        )
        self.run_marker_path = Path(
            args.run_marker_path or self.output_dir / "health_monitor_run.json"
        )
        self.stop_marker_path = Path(
            args.stop_marker_path or self.output_dir / "health_monitor_stop.json"
        )

        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.csv_file.flush()
        self.sync_csv(force=True)
        self.event_file = self.event_path.open("a", encoding="utf-8")
        self.event_context_file = None
        self.event_context_writer = None
        if args.event_context_csv:
            context_fields = ["event_id", "context_phase"] + CSV_FIELDS
            self.event_context_file = self.event_context_path.open(
                "w", newline="", encoding="utf-8"
            )
            self.event_context_writer = csv.DictWriter(
                self.event_context_file, fieldnames=context_fields
            )
            self.event_context_writer.writeheader()
            self.event_context_file.flush()
        self.write_run_marker()

        self.subscribe_if_configured(PoseStamped, args.raw_topic, self.on_raw_pose)
        self.subscribe_if_configured(
            PoseStamped, args.continuous_topic, self.on_continuous_pose
        )
        self.subscribe_if_configured(PoseStamped, args.filtered_topic, self.on_filtered_pose)
        self.subscribe_if_configured(PoseStamped, args.beta_topic, self.on_beta_pose)
        self.subscribe_if_configured(PoseStamped, args.stable_topic, self.on_stable_pose)
        self.subscribe_if_configured(Float32MultiArray, args.quality_topic, self.on_quality)
        self.subscribe_if_configured(
            Float32MultiArray, args.filter_status_topic, self.on_filter_status
        )
        self.subscribe_if_configured(
            Float32MultiArray,
            args.beta_filter_status_topic,
            self.on_beta_filter_status,
        )
        self.subscribe_if_configured(
            Float32MultiArray,
            args.stable_filter_status_topic,
            self.on_stable_filter_status,
        )
        self.subscribe_if_configured(
            Float32MultiArray,
            args.continuous_status_topic,
            self.on_continuous_status,
        )
        self.subscribe_if_configured(
            Float32MultiArray,
            args.yaw_fusion_status_topic,
            self.on_yaw_fusion_status,
        )
        self.timer = self.create_timer(args.interval, self.sample)

        self.get_logger().info(
            f"health monitor logging csv={self.csv_path} summary={self.summary_path} "
            f"events={self.event_path} raw={args.raw_topic} continuous={args.continuous_topic} "
            f"filtered={args.filtered_topic} "
            f"beta={args.beta_topic} stable={args.stable_topic} quality={args.quality_topic} "
            f"filter_status={args.filter_status_topic} beta_status={args.beta_filter_status_topic} "
            f"stable_status={args.stable_filter_status_topic} continuous_status={args.continuous_status_topic} "
            f"yaw_fusion_status={args.yaw_fusion_status_topic}"
        )
        self.get_logger().info(
            f"health monitor markers run={self.run_marker_path} stop={self.stop_marker_path} "
            f"fsync_interval={args.fsync_interval:.1f}s"
        )

    def subscribe_if_configured(self, msg_type, topic: str, callback) -> None:
        if not topic:
            return
        self.create_subscription(msg_type, topic, callback, 20)

    def sync_csv(self, force: bool = False) -> None:
        if self.args.fsync_interval <= 0.0 and not force:
            return
        now = time.monotonic()
        if force or now - self.last_csv_fsync >= self.args.fsync_interval:
            os.fsync(self.csv_file.fileno())
            self.last_csv_fsync = now

    def write_run_marker(self) -> None:
        atomic_write_json(
            self.run_marker_path,
            {
                "event": "health_monitor_started",
                "started_at": datetime.now().isoformat(timespec="seconds"),
                "run_id": os.environ.get("RUN_ID", ""),
                "run_reason": os.environ.get("RUN_REASON", ""),
                "pid": os.getpid(),
                "hostname": socket.gethostname(),
                "output_dir": str(self.output_dir),
                "csv_path": str(self.csv_path),
                "summary_path": str(self.summary_path),
                "event_path": str(self.event_path),
                "event_context_path": (
                    str(self.event_context_path) if self.args.event_context_csv else ""
                ),
                "stop_marker_path": str(self.stop_marker_path),
                "raw_topic": self.args.raw_topic,
                "continuous_topic": self.args.continuous_topic,
                "filtered_topic": self.args.filtered_topic,
                "beta_topic": self.args.beta_topic,
                "stable_topic": self.args.stable_topic,
                "quality_topic": self.args.quality_topic,
                "filter_status_topic": self.args.filter_status_topic,
                "beta_filter_status_topic": self.args.beta_filter_status_topic,
                "stable_filter_status_topic": self.args.stable_filter_status_topic,
                "continuous_status_topic": self.args.continuous_status_topic,
                "yaw_fusion_status_topic": self.args.yaw_fusion_status_topic,
                "interval_sec": self.args.interval,
                "window_sec": self.args.window,
                "summary_interval_sec": self.args.summary_interval,
                "fsync_interval_sec": self.args.fsync_interval,
                "event_pre_sec": self.args.event_pre_sec,
                "event_post_sec": self.args.event_post_sec,
                "spike_raw_step_m": self.args.spike_raw_step,
                "spike_filtered_step_m": self.args.spike_filtered_step,
                "spike_map_jump_m": self.args.spike_map_jump,
                "spike_fitness": self.args.spike_fitness,
                "sustained_motion_sec": self.args.sustained_motion_sec,
            },
        )

    def write_stop_marker(self, reason: str) -> None:
        atomic_write_json(
            self.stop_marker_path,
            {
                "event": "health_monitor_stopped",
                "stopped_at": datetime.now().isoformat(timespec="seconds"),
                "run_id": os.environ.get("RUN_ID", ""),
                "run_reason": os.environ.get("RUN_REASON", ""),
                "pid": os.getpid(),
                "reason": reason,
                "sample_count": len(self.rows),
                "event_count": self.event_count,
                "duration_sec": time.monotonic() - self.start_time,
                "csv_path": str(self.csv_path),
                "summary_path": str(self.summary_path),
                "event_path": str(self.event_path),
            },
        )

    def on_raw_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        pose = (time.monotonic(), float(p.x), float(p.y), float(p.z), yaw_from_pose(msg))
        step_xy, speed_xy, yaw_rate = pose_step(self.callback_previous_raw_pose, pose)
        self.raw_pose = pose
        self.callback_previous_raw_pose = pose
        if math.isfinite(step_xy) and step_xy >= self.args.spike_raw_step:
            self.start_event(
                "raw_pose_step",
                pose[0],
                {
                    "raw_step_xy_m": step_xy,
                    "raw_speed_xy_mps": speed_xy,
                    "raw_yaw_rate_degps": yaw_rate,
                    "threshold_m": self.args.spike_raw_step,
                },
            )

    def on_continuous_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        pose = (time.monotonic(), float(p.x), float(p.y), float(p.z), yaw_from_pose(msg))
        self.continuous_pose = pose
        self.callback_previous_continuous_pose = pose

    def on_filtered_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        pose = (time.monotonic(), float(p.x), float(p.y), float(p.z), yaw_from_pose(msg))
        step_xy, speed_xy, yaw_rate = pose_step(self.callback_previous_filtered_pose, pose)
        self.filtered_pose = pose
        self.callback_previous_filtered_pose = pose
        if math.isfinite(step_xy) and step_xy >= self.args.spike_filtered_step:
            self.start_event(
                "filtered_pose_step",
                pose[0],
                {
                    "filtered_step_xy_m": step_xy,
                    "filtered_speed_xy_mps": speed_xy,
                    "filtered_yaw_rate_degps": yaw_rate,
                    "threshold_m": self.args.spike_filtered_step,
                },
            )

    def on_beta_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        pose = (time.monotonic(), float(p.x), float(p.y), float(p.z), yaw_from_pose(msg))
        step_xy, speed_xy, yaw_rate = pose_step(self.callback_previous_beta_pose, pose)
        self.beta_pose = pose
        self.callback_previous_beta_pose = pose
        if math.isfinite(step_xy) and step_xy >= self.args.spike_filtered_step:
            self.start_event(
                "beta_pose_step",
                pose[0],
                {
                    "beta_step_xy_m": step_xy,
                    "beta_speed_xy_mps": speed_xy,
                    "beta_yaw_rate_degps": yaw_rate,
                    "threshold_m": self.args.spike_filtered_step,
                },
            )

    def on_stable_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        pose = (time.monotonic(), float(p.x), float(p.y), float(p.z), yaw_from_pose(msg))
        step_xy, speed_xy, yaw_rate = pose_step(self.callback_previous_stable_pose, pose)
        self.stable_pose = pose
        self.callback_previous_stable_pose = pose
        if math.isfinite(step_xy) and step_xy >= self.args.spike_filtered_step:
            self.start_event(
                "stable_pose_step",
                pose[0],
                {
                    "stable_step_xy_m": step_xy,
                    "stable_speed_xy_mps": speed_xy,
                    "stable_yaw_rate_degps": yaw_rate,
                    "threshold_m": self.args.spike_filtered_step,
                },
            )

    def on_quality(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        values = [float(value) for value in msg.data]
        self.quality = (now, values)
        accepted = values[0] if len(values) > 0 else math.nan
        fitness = values[2] if len(values) > 2 else math.nan
        quality_jump = values[7] if len(values) > 7 else math.nan
        reason_code = values[10] if len(values) > 10 else math.nan
        if math.isfinite(quality_jump) and quality_jump >= self.args.spike_map_jump:
            self.start_event(
                "map_output_jump",
                now,
                {
                    "map_quality_jump_xy_m": quality_jump,
                    "threshold_m": self.args.spike_map_jump,
                    "fitness": fitness,
                    "reason_code": reason_code,
                },
            )
        if math.isfinite(fitness) and fitness >= self.args.spike_fitness:
            self.start_event(
                "map_bad_fitness",
                now,
                {
                    "fitness": fitness,
                    "threshold": self.args.spike_fitness,
                    "accepted": accepted,
                    "reason_code": reason_code,
                },
            )
        if math.isfinite(accepted) and accepted < 0.5:
            self.start_event(
                "map_rejected",
                now,
                {
                    "fitness": fitness,
                    "reason_code": reason_code,
                    "scan_points": values[5] if len(values) > 5 else math.nan,
                },
            )

    def on_filter_status(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        values = [float(value) for value in msg.data]
        self.filter_status = (now, values)
        accepted = values[0] if len(values) > 0 else math.nan
        raw_step = values[5] if len(values) > 5 else math.nan
        recovered = values[13] if len(values) > 13 else math.nan
        motion_factor = values[14] if len(values) > 14 else math.nan
        if math.isfinite(accepted) and accepted < 0.5:
            self.start_event(
                "filter_rejected",
                now,
                {
                    "filter_raw_step_xy_m": raw_step,
                    "filter_motion_factor": motion_factor,
                    "consecutive_rejects": values[12] if len(values) > 12 else math.nan,
                },
            )
        if math.isfinite(recovered) and recovered > 0.5:
            self.start_event(
                "filter_recovered",
                now,
                {
                    "filter_raw_step_xy_m": raw_step,
                    "filter_motion_factor": motion_factor,
                    "consecutive_rejects": values[12] if len(values) > 12 else math.nan,
                },
            )

    def on_beta_filter_status(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        values = [float(value) for value in msg.data]
        self.beta_filter_status = (now, values)
        accepted = values[0] if len(values) > 0 else math.nan
        raw_step = values[5] if len(values) > 5 else math.nan
        recovered = values[13] if len(values) > 13 else math.nan
        motion_factor = values[14] if len(values) > 14 else math.nan
        reason_code = values[17] if len(values) > 17 else math.nan
        hold_state = values[18] if len(values) > 18 else math.nan
        confirm_count = values[19] if len(values) > 19 else math.nan
        if math.isfinite(accepted) and accepted < 0.5:
            self.start_event(
                "beta_filter_rejected",
                now,
                {
                    "beta_filter_raw_step_xy_m": raw_step,
                    "beta_filter_motion_factor": motion_factor,
                    "beta_filter_reason_code": reason_code,
                    "beta_filter_hold_state_code": hold_state,
                    "beta_filter_confirm_count": confirm_count,
                },
            )
        if math.isfinite(recovered) and recovered > 0.5:
            self.start_event(
                "beta_filter_recovered",
                now,
                {
                    "beta_filter_raw_step_xy_m": raw_step,
                    "beta_filter_motion_factor": motion_factor,
                    "beta_filter_reason_code": reason_code,
                    "beta_filter_hold_state_code": hold_state,
                },
            )

    def on_stable_filter_status(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        values = [float(value) for value in msg.data]
        self.stable_filter_status = (now, values)
        accepted = values[0] if len(values) > 0 else math.nan
        raw_step = values[5] if len(values) > 5 else math.nan
        recovered = values[13] if len(values) > 13 else math.nan
        motion_factor = values[14] if len(values) > 14 else math.nan
        reason_code = values[17] if len(values) > 17 else math.nan
        hold_state = values[18] if len(values) > 18 else math.nan
        confirm_count = values[19] if len(values) > 19 else math.nan
        if math.isfinite(accepted) and accepted < 0.5:
            self.start_event(
                "stable_filter_rejected",
                now,
                {
                    "stable_filter_raw_step_xy_m": raw_step,
                    "stable_filter_motion_factor": motion_factor,
                    "stable_filter_reason_code": reason_code,
                    "stable_filter_hold_state_code": hold_state,
                    "stable_filter_confirm_count": confirm_count,
                },
            )
        if math.isfinite(recovered) and recovered > 0.5:
            self.start_event(
                "stable_filter_recovered",
                now,
                {
                    "stable_filter_raw_step_xy_m": raw_step,
                    "stable_filter_motion_factor": motion_factor,
                    "stable_filter_reason_code": reason_code,
                    "stable_filter_hold_state_code": hold_state,
                },
            )

    def on_continuous_status(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        values = [float(value) for value in msg.data]
        self.continuous_status = (now, values)

    def on_yaw_fusion_status(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        values = [float(value) for value in msg.data]
        self.yaw_fusion_status = (now, values)

    def start_event(self, reason: str, stamp: float, trigger: dict[str, Any]) -> None:
        last_stamp = self.last_event_by_reason.get(reason)
        if last_stamp is not None and stamp - last_stamp < self.args.event_cooldown_sec:
            return
        self.last_event_by_reason[reason] = stamp
        self.event_count += 1
        trigger_elapsed = stamp - self.start_time
        event = {
            "event_id": self.event_count,
            "reason": reason,
            "trigger_monotonic": stamp,
            "trigger_elapsed_sec": trigger_elapsed,
            "trigger_wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "trigger": trigger,
            "pre_rows": [
                row
                for row in self.event_buffer
                if 0.0 <= trigger_elapsed - float(row.get("elapsed_sec", 0.0))
                <= self.args.event_pre_sec
            ],
        }
        self.open_events.append(event)

    def summarize_event_rows(self, rows: list[dict[str, float | str]]) -> dict[str, Any]:
        numeric_rows: list[dict[str, float]] = []
        for row in rows:
            numeric_rows.append(
                {
                    key: float(value)
                    for key, value in row.items()
                    if key != "time_iso" and isinstance(value, (int, float))
                }
            )
        if not numeric_rows:
            return {"sample_count": 0}
        first = numeric_rows[0]
        last = numeric_rows[-1]
        return {
            "sample_count": len(numeric_rows),
            "elapsed_start_sec": first.get("elapsed_sec", math.nan),
            "elapsed_end_sec": last.get("elapsed_sec", math.nan),
            "max_temp_c": series_stats(numeric_rows, "max_temp_c"),
            "load1": series_stats(numeric_rows, "load1"),
            "cpu_freq_avg_mhz": series_stats(numeric_rows, "cpu_freq_avg_mhz"),
            "raw_step_xy_m": series_stats(numeric_rows, "raw_step_xy_m"),
            "filtered_step_xy_m": series_stats(numeric_rows, "filtered_step_xy_m"),
            "stable_step_xy_m": series_stats(numeric_rows, "stable_step_xy_m"),
            "raw_window_xy_p2p_m": series_stats(numeric_rows, "raw_window_xy_p2p_m"),
            "filtered_window_xy_p2p_m": series_stats(numeric_rows, "filtered_window_xy_p2p_m"),
            "stable_window_xy_p2p_m": series_stats(
                numeric_rows,
                "stable_window_xy_p2p_m",
            ),
            "static_context_raw_xy_p2p_m": series_stats(
                numeric_rows, "static_context_raw_xy_p2p_m"
            ),
            "static_context_filtered_xy_p2p_m": series_stats(
                numeric_rows, "static_context_filtered_xy_p2p_m"
            ),
            "map_quality_fitness": series_stats(numeric_rows, "map_quality_fitness"),
            "map_quality_jump_xy_m": series_stats(numeric_rows, "map_quality_jump_xy_m"),
            "map_quality_align_ms": series_stats(numeric_rows, "map_quality_align_ms"),
            "map_quality_scan_age_sec": series_stats(numeric_rows, "map_quality_scan_age_sec"),
            "map_quality_odom_age_sec": series_stats(numeric_rows, "map_quality_odom_age_sec"),
            "map_quality_scan_odom_delta_sec": series_stats(
                numeric_rows, "map_quality_scan_odom_delta_sec"
            ),
            "map_quality_preprocess_ms": series_stats(
                numeric_rows, "map_quality_preprocess_ms"
            ),
            "map_quality_total_ms": series_stats(numeric_rows, "map_quality_total_ms"),
            "filter_status_alpha": series_stats(numeric_rows, "filter_status_alpha"),
            "filter_status_motion_factor": series_stats(
                numeric_rows, "filter_status_motion_factor"
            ),
            "stable_filter_status_alpha": series_stats(
                numeric_rows,
                "stable_filter_status_alpha",
            ),
            "stable_filter_status_motion_factor": series_stats(
                numeric_rows,
                "stable_filter_status_motion_factor",
            ),
            "spike_candidate_count": sum(
                1 for row in numeric_rows if row.get("spike_candidate", 0.0) > 0.5
            ),
            "sustained_motion_count": sum(
                1 for row in numeric_rows if row.get("sustained_motion", 0.0) > 0.5
            ),
        }

    def close_finished_events(self, now: float) -> None:
        remaining: list[dict[str, Any]] = []
        for event in self.open_events:
            trigger_elapsed = float(event["trigger_elapsed_sec"])
            if now - self.start_time < trigger_elapsed + self.args.event_post_sec:
                remaining.append(event)
                continue

            all_rows = [
                row
                for row in self.event_buffer
                if trigger_elapsed - self.args.event_pre_sec
                <= float(row.get("elapsed_sec", math.nan))
                <= trigger_elapsed + self.args.event_post_sec
            ]
            pre_rows = [
                row
                for row in all_rows
                if float(row.get("elapsed_sec", math.nan)) <= trigger_elapsed
            ]
            post_rows = [
                row
                for row in all_rows
                if float(row.get("elapsed_sec", math.nan)) > trigger_elapsed
            ]
            payload = {
                "event_id": event["event_id"],
                "reason": event["reason"],
                "trigger_wall_time": event["trigger_wall_time"],
                "trigger_elapsed_sec": trigger_elapsed,
                "trigger": event["trigger"],
                "pre_summary": self.summarize_event_rows(pre_rows),
                "post_summary": self.summarize_event_rows(post_rows),
                "window_summary": self.summarize_event_rows(all_rows),
            }
            self.event_file.write(json.dumps(json_safe(payload), ensure_ascii=False, allow_nan=False))
            self.event_file.write("\n")
            self.event_file.flush()
            os.fsync(self.event_file.fileno())
            if self.event_context_writer:
                for phase, rows in (("pre", pre_rows), ("post", post_rows)):
                    for row in rows:
                        context_row = {"event_id": event["event_id"], "context_phase": phase}
                        context_row.update(row)
                        self.event_context_writer.writerow(context_row)
                self.event_context_file.flush()
        self.open_events = remaining

    def trim_window(self, window: Deque[PoseSample], now: float) -> None:
        cutoff = now - self.args.window
        while window and window[0][0] < cutoff:
            window.popleft()

    def compute_motion_factor(self, raw_speed: float, raw_yaw_rate: float, filtered_speed: float, filtered_yaw_rate: float) -> float:
        speed = filtered_speed if math.isfinite(filtered_speed) else raw_speed
        yaw_rate = filtered_yaw_rate if math.isfinite(filtered_yaw_rate) else raw_yaw_rate
        speed_factor = ramp(speed, self.args.static_speed, self.args.full_speed)
        yaw_factor = ramp(abs(yaw_rate), self.args.static_yaw_rate_deg, self.args.full_yaw_rate_deg)
        factors = [value for value in (speed_factor, yaw_factor) if math.isfinite(value)]
        return max(factors) if factors else math.nan

    def append_windows(
        self,
        now: float,
        is_stationary: bool,
        sustained_motion: bool,
        spike_candidate: bool,
    ) -> None:
        if self.raw_pose:
            self.raw_window.append(self.raw_pose)
        if self.continuous_pose:
            self.continuous_window.append(self.continuous_pose)
        if self.filtered_pose:
            self.filtered_window.append(self.filtered_pose)
        if self.beta_pose:
            self.beta_window.append(self.beta_pose)
        if self.stable_pose:
            self.stable_window.append(self.stable_pose)

        if is_stationary:
            if self.raw_pose:
                self.stationary_raw_window.append(self.raw_pose)
            if self.filtered_pose:
                self.stationary_filtered_window.append(self.filtered_pose)
        else:
            self.stationary_raw_window.clear()
            self.stationary_filtered_window.clear()

        if not sustained_motion:
            if self.raw_pose:
                self.static_context_raw_window.append(self.raw_pose)
            if self.filtered_pose:
                self.static_context_filtered_window.append(self.filtered_pose)
        else:
            self.static_context_raw_window.clear()
            self.static_context_filtered_window.clear()

        self.trim_window(self.raw_window, now)
        self.trim_window(self.continuous_window, now)
        self.trim_window(self.filtered_window, now)
        self.trim_window(self.beta_window, now)
        self.trim_window(self.stable_window, now)
        self.trim_window(self.stationary_raw_window, now)
        self.trim_window(self.stationary_filtered_window, now)
        self.trim_window(self.static_context_raw_window, now)
        self.trim_window(self.static_context_filtered_window, now)

    def sample(self) -> None:
        now = time.monotonic()
        temp_c = read_max_temperature_c()
        load1, load5, load15 = read_loadavg()
        freq_min, freq_max, freq_avg = read_cpu_freq_mhz()

        raw_step, raw_speed, raw_yaw_rate = pose_step(self.previous_raw_pose, self.raw_pose)
        continuous_step, continuous_speed, continuous_yaw_rate = pose_step(
            self.previous_continuous_pose,
            self.continuous_pose,
        )
        filtered_step, filtered_speed, filtered_yaw_rate = pose_step(
            self.previous_filtered_pose,
            self.filtered_pose,
        )
        beta_step, beta_speed, beta_yaw_rate = pose_step(
            self.previous_beta_pose,
            self.beta_pose,
        )
        stable_step, stable_speed, stable_yaw_rate = pose_step(
            self.previous_stable_pose,
            self.stable_pose,
        )
        motion_factor = self.compute_motion_factor(
            raw_speed,
            raw_yaw_rate,
            filtered_speed,
            filtered_yaw_rate,
        )

        accepted = converged = fitness = quality_jump = scan_points = map_points = math.nan
        reason_code = align_ms = raw_scan_points = quality_scan_age = math.nan
        quality_odom_age = output_jump_yaw = map_to_odom_delta_xy = math.nan
        map_to_odom_delta_yaw = overlap_ratio = math.nan
        scan_odom_delta = scan_reused = odom_buffer_size = math.nan
        localize_busy = preprocess_ms = quality_total_ms = math.nan
        quality_age = math.nan
        if self.quality:
            stamp, values = self.quality
            quality_age = now - stamp
            accepted = values[0] if len(values) > 0 else math.nan
            converged = values[1] if len(values) > 1 else math.nan
            fitness = values[2] if len(values) > 2 else math.nan
            scan_points = values[5] if len(values) > 5 else math.nan
            map_points = values[6] if len(values) > 6 else math.nan
            quality_jump = values[7] if len(values) > 7 else math.nan
            reason_code = values[10] if len(values) > 10 else math.nan
            align_ms = values[11] if len(values) > 11 else math.nan
            raw_scan_points = values[12] if len(values) > 12 else math.nan
            quality_scan_age = values[13] if len(values) > 13 else math.nan
            quality_odom_age = values[14] if len(values) > 14 else math.nan
            output_jump_yaw = values[15] if len(values) > 15 else math.nan
            map_to_odom_delta_xy = values[16] if len(values) > 16 else math.nan
            map_to_odom_delta_yaw = values[17] if len(values) > 17 else math.nan
            overlap_ratio = values[18] if len(values) > 18 else math.nan
            scan_odom_delta = values[19] if len(values) > 19 else math.nan
            scan_reused = values[20] if len(values) > 20 else math.nan
            odom_buffer_size = values[21] if len(values) > 21 else math.nan
            localize_busy = values[22] if len(values) > 22 else math.nan
            preprocess_ms = values[23] if len(values) > 23 else math.nan
            quality_total_ms = values[24] if len(values) > 24 else math.nan

        (
            filter_accepted,
            filter_raw_step,
            filter_speed,
            filter_alpha,
            filter_reject_count,
            filter_consecutive_rejects,
            filter_recovered,
            filter_motion_factor,
            filter_yaw_rate,
            filter_reason_code,
            filter_hold_state,
            filter_confirm_count,
            filter_status_age,
        ) = (math.nan,) * 13
        if self.filter_status:
            stamp, values = self.filter_status
            filter_status_age = now - stamp
            filter_accepted = values[0] if len(values) > 0 else math.nan
            filter_raw_step = values[5] if len(values) > 5 else math.nan
            filter_speed = values[6] if len(values) > 6 else math.nan
            filter_alpha = values[7] if len(values) > 7 else math.nan
            filter_reject_count = values[10] if len(values) > 10 else math.nan
            filter_consecutive_rejects = values[12] if len(values) > 12 else math.nan
            filter_recovered = values[13] if len(values) > 13 else math.nan
            filter_motion_factor = values[14] if len(values) > 14 else math.nan
            filter_yaw_rate = values[15] if len(values) > 15 else math.nan
            filter_reason_code = values[17] if len(values) > 17 else math.nan
            filter_hold_state = values[18] if len(values) > 18 else math.nan
            filter_confirm_count = values[19] if len(values) > 19 else math.nan

        (
            beta_filter_accepted,
            beta_filter_raw_step,
            beta_filter_speed,
            beta_filter_alpha,
            beta_filter_reject_count,
            beta_filter_consecutive_rejects,
            beta_filter_recovered,
            beta_filter_motion_factor,
            beta_filter_yaw_rate,
            beta_filter_reason_code,
            beta_filter_hold_state,
            beta_filter_confirm_count,
            beta_filter_status_age,
        ) = (math.nan,) * 13
        if self.beta_filter_status:
            stamp, values = self.beta_filter_status
            beta_filter_status_age = now - stamp
            beta_filter_accepted = values[0] if len(values) > 0 else math.nan
            beta_filter_raw_step = values[5] if len(values) > 5 else math.nan
            beta_filter_speed = values[6] if len(values) > 6 else math.nan
            beta_filter_alpha = values[7] if len(values) > 7 else math.nan
            beta_filter_reject_count = values[10] if len(values) > 10 else math.nan
            beta_filter_consecutive_rejects = values[12] if len(values) > 12 else math.nan
            beta_filter_recovered = values[13] if len(values) > 13 else math.nan
            beta_filter_motion_factor = values[14] if len(values) > 14 else math.nan
            beta_filter_yaw_rate = values[15] if len(values) > 15 else math.nan
            beta_filter_reason_code = values[17] if len(values) > 17 else math.nan
            beta_filter_hold_state = values[18] if len(values) > 18 else math.nan
            beta_filter_confirm_count = values[19] if len(values) > 19 else math.nan

        (
            stable_filter_accepted,
            stable_filter_raw_step,
            stable_filter_speed,
            stable_filter_alpha,
            stable_filter_reject_count,
            stable_filter_consecutive_rejects,
            stable_filter_recovered,
            stable_filter_motion_factor,
            stable_filter_yaw_rate,
            stable_filter_reason_code,
            stable_filter_hold_state,
            stable_filter_confirm_count,
            stable_filter_status_age,
        ) = (math.nan,) * 13
        if self.stable_filter_status:
            stamp, values = self.stable_filter_status
            stable_filter_status_age = now - stamp
            stable_filter_accepted = values[0] if len(values) > 0 else math.nan
            stable_filter_raw_step = values[5] if len(values) > 5 else math.nan
            stable_filter_speed = values[6] if len(values) > 6 else math.nan
            stable_filter_alpha = values[7] if len(values) > 7 else math.nan
            stable_filter_reject_count = values[10] if len(values) > 10 else math.nan
            stable_filter_consecutive_rejects = values[12] if len(values) > 12 else math.nan
            stable_filter_recovered = values[13] if len(values) > 13 else math.nan
            stable_filter_motion_factor = values[14] if len(values) > 14 else math.nan
            stable_filter_yaw_rate = values[15] if len(values) > 15 else math.nan
            stable_filter_reason_code = values[17] if len(values) > 17 else math.nan
            stable_filter_hold_state = values[18] if len(values) > 18 else math.nan
            stable_filter_confirm_count = values[19] if len(values) > 19 else math.nan

        (
            continuous_ready,
            continuous_map_update_accepted,
            continuous_correction_alpha,
            continuous_correction_age,
            continuous_odom_age,
            continuous_stamp_delta,
            continuous_candidate_delta_xy,
            continuous_candidate_delta_yaw,
            continuous_quality_fitness,
            continuous_update_count,
            continuous_rejected_count,
            continuous_match_mode_code,
            continuous_reason_code,
            continuous_status_age,
        ) = (math.nan,) * 14
        if self.continuous_status:
            stamp, values = self.continuous_status
            continuous_status_age = now - stamp
            continuous_ready = values[1] if len(values) > 1 else math.nan
            continuous_map_update_accepted = values[2] if len(values) > 2 else math.nan
            continuous_correction_alpha = values[3] if len(values) > 3 else math.nan
            continuous_correction_age = values[4] if len(values) > 4 else math.nan
            continuous_odom_age = values[5] if len(values) > 5 else math.nan
            continuous_stamp_delta = values[6] if len(values) > 6 else math.nan
            continuous_candidate_delta_xy = values[7] if len(values) > 7 else math.nan
            continuous_candidate_delta_yaw = values[8] if len(values) > 8 else math.nan
            continuous_quality_fitness = values[11] if len(values) > 11 else math.nan
            continuous_update_count = values[13] if len(values) > 13 else math.nan
            continuous_rejected_count = values[14] if len(values) > 14 else math.nan
            continuous_match_mode_code = values[15] if len(values) > 15 else math.nan
            continuous_reason_code = values[19] if len(values) > 19 else math.nan

        (
            yaw_fusion_enabled,
            yaw_fusion_ready,
            yaw_fusion_source_code,
            yaw_fusion_fast_age,
            yaw_fusion_map_age,
            yaw_fusion_map_ref_yaw,
            yaw_fusion_fast_yaw,
            yaw_fusion_fused_yaw,
            yaw_fusion_delta_yaw,
            yaw_fusion_map_correction_alpha,
            yaw_fusion_status_age,
        ) = (math.nan,) * 11
        if self.yaw_fusion_status:
            stamp, values = self.yaw_fusion_status
            yaw_fusion_status_age = now - stamp
            yaw_fusion_enabled = values[0] if len(values) > 0 else math.nan
            yaw_fusion_ready = values[1] if len(values) > 1 else math.nan
            yaw_fusion_source_code = values[2] if len(values) > 2 else math.nan
            yaw_fusion_fast_age = values[3] if len(values) > 3 else math.nan
            yaw_fusion_map_age = values[4] if len(values) > 4 else math.nan
            yaw_fusion_map_ref_yaw = values[5] if len(values) > 5 else math.nan
            yaw_fusion_fast_yaw = values[6] if len(values) > 6 else math.nan
            yaw_fusion_fused_yaw = values[7] if len(values) > 7 else math.nan
            yaw_fusion_delta_yaw = values[8] if len(values) > 8 else math.nan
            yaw_fusion_map_correction_alpha = values[9] if len(values) > 9 else math.nan

        spike_candidate = bool(
            (math.isfinite(raw_step) and raw_step >= self.args.spike_raw_step)
            or (
                math.isfinite(filtered_step)
                and filtered_step >= self.args.spike_filtered_step
            )
            or (
                math.isfinite(beta_step)
                and beta_step >= self.args.spike_filtered_step
            )
            or (
                math.isfinite(stable_step)
                and stable_step >= self.args.spike_filtered_step
            )
            or (
                math.isfinite(quality_jump)
                and quality_jump >= self.args.spike_map_jump
            )
            or (
                math.isfinite(fitness)
                and fitness >= self.args.spike_fitness
            )
            or (math.isfinite(filter_accepted) and filter_accepted < 0.5)
            or (math.isfinite(filter_recovered) and filter_recovered > 0.5)
            or (math.isfinite(stable_filter_accepted) and stable_filter_accepted < 0.5)
            or (math.isfinite(stable_filter_recovered) and stable_filter_recovered > 0.5)
        )

        motion_now = bool(
            math.isfinite(motion_factor)
            and motion_factor > self.args.stationary_factor
        )
        if motion_now:
            if self.motion_started_at is None:
                self.motion_started_at = now
        else:
            self.motion_started_at = None
        sustained_motion = bool(
            self.motion_started_at is not None
            and now - self.motion_started_at >= self.args.sustained_motion_sec
        )
        is_stationary = bool(
            math.isfinite(motion_factor)
            and motion_factor <= self.args.stationary_factor
            and (not self.filtered_pose or now - self.filtered_pose[0] <= self.args.max_pose_age)
        )

        self.append_windows(now, is_stationary, sustained_motion, spike_candidate)

        raw_x = raw_y = raw_z = raw_yaw = math.nan
        raw_age = math.nan
        if self.raw_pose:
            stamp, raw_x, raw_y, raw_z, raw_yaw = self.raw_pose
            raw_age = now - stamp

        continuous_x = continuous_y = continuous_z = continuous_yaw = math.nan
        continuous_age = math.nan
        if self.continuous_pose:
            stamp, continuous_x, continuous_y, continuous_z, continuous_yaw = self.continuous_pose
            continuous_age = now - stamp

        filtered_x = filtered_y = filtered_z = filtered_yaw = math.nan
        filtered_age = math.nan
        if self.filtered_pose:
            stamp, filtered_x, filtered_y, filtered_z, filtered_yaw = self.filtered_pose
            filtered_age = now - stamp

        beta_x = beta_y = beta_z = beta_yaw = math.nan
        beta_age = math.nan
        if self.beta_pose:
            stamp, beta_x, beta_y, beta_z, beta_yaw = self.beta_pose
            beta_age = now - stamp

        stable_x = stable_y = stable_z = stable_yaw = math.nan
        stable_age = math.nan
        if self.stable_pose:
            stamp, stable_x, stable_y, stable_z, stable_yaw = self.stable_pose
            stable_age = now - stamp

        delta_xy = math.nan
        delta_yaw = math.nan
        raw_continuous_delta_xy = math.nan
        continuous_filtered_delta_xy = math.nan
        raw_beta_delta_xy = math.nan
        filtered_beta_delta_xy = math.nan
        raw_stable_delta_xy = math.nan
        filtered_stable_delta_xy = math.nan
        if all(math.isfinite(value) for value in (raw_x, raw_y, filtered_x, filtered_y)):
            delta_xy = math.hypot(raw_x - filtered_x, raw_y - filtered_y)
        if math.isfinite(raw_yaw) and math.isfinite(filtered_yaw):
            delta_yaw = math.degrees(abs(shortest_angle_rad(raw_yaw - filtered_yaw)))
        if all(math.isfinite(value) for value in (raw_x, raw_y, continuous_x, continuous_y)):
            raw_continuous_delta_xy = math.hypot(raw_x - continuous_x, raw_y - continuous_y)
        if all(math.isfinite(value) for value in (continuous_x, continuous_y, filtered_x, filtered_y)):
            continuous_filtered_delta_xy = math.hypot(
                continuous_x - filtered_x,
                continuous_y - filtered_y,
            )
        if all(math.isfinite(value) for value in (raw_x, raw_y, beta_x, beta_y)):
            raw_beta_delta_xy = math.hypot(raw_x - beta_x, raw_y - beta_y)
        if all(math.isfinite(value) for value in (filtered_x, filtered_y, beta_x, beta_y)):
            filtered_beta_delta_xy = math.hypot(filtered_x - beta_x, filtered_y - beta_y)
        if all(math.isfinite(value) for value in (raw_x, raw_y, stable_x, stable_y)):
            raw_stable_delta_xy = math.hypot(raw_x - stable_x, raw_y - stable_y)
        if all(math.isfinite(value) for value in (filtered_x, filtered_y, stable_x, stable_y)):
            filtered_stable_delta_xy = math.hypot(
                filtered_x - stable_x,
                filtered_y - stable_y,
            )

        raw_x_p2p, raw_y_p2p, raw_xy_p2p = window_p2p(self.raw_window)
        filtered_x_p2p, filtered_y_p2p, filtered_xy_p2p = window_p2p(self.filtered_window)
        beta_x_p2p, beta_y_p2p, beta_xy_p2p = window_p2p(self.beta_window)
        stable_x_p2p, stable_y_p2p, stable_xy_p2p = window_p2p(self.stable_window)
        stationary_raw_x_p2p, stationary_raw_y_p2p, stationary_raw_xy_p2p = window_p2p(
            self.stationary_raw_window
        )
        (
            stationary_filtered_x_p2p,
            stationary_filtered_y_p2p,
            stationary_filtered_xy_p2p,
        ) = window_p2p(self.stationary_filtered_window)
        _, _, static_context_raw_xy_p2p = window_p2p(self.static_context_raw_window)
        _, _, static_context_filtered_xy_p2p = window_p2p(
            self.static_context_filtered_window
        )

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
            "raw_yaw_deg": math.degrees(raw_yaw) if math.isfinite(raw_yaw) else math.nan,
            "continuous_x": continuous_x,
            "continuous_y": continuous_y,
            "continuous_z": continuous_z,
            "continuous_yaw_deg": math.degrees(continuous_yaw) if math.isfinite(continuous_yaw) else math.nan,
            "filtered_x": filtered_x,
            "filtered_y": filtered_y,
            "filtered_z": filtered_z,
            "filtered_yaw_deg": math.degrees(filtered_yaw) if math.isfinite(filtered_yaw) else math.nan,
            "beta_x": beta_x,
            "beta_y": beta_y,
            "beta_z": beta_z,
            "beta_yaw_deg": math.degrees(beta_yaw) if math.isfinite(beta_yaw) else math.nan,
            "stable_x": stable_x,
            "stable_y": stable_y,
            "stable_z": stable_z,
            "stable_yaw_deg": math.degrees(stable_yaw) if math.isfinite(stable_yaw) else math.nan,
            "raw_filtered_delta_xy_m": delta_xy,
            "raw_filtered_delta_yaw_deg": delta_yaw,
            "raw_continuous_delta_xy_m": raw_continuous_delta_xy,
            "continuous_filtered_delta_xy_m": continuous_filtered_delta_xy,
            "raw_beta_delta_xy_m": raw_beta_delta_xy,
            "filtered_beta_delta_xy_m": filtered_beta_delta_xy,
            "raw_stable_delta_xy_m": raw_stable_delta_xy,
            "filtered_stable_delta_xy_m": filtered_stable_delta_xy,
            "raw_step_xy_m": raw_step,
            "raw_speed_xy_mps": raw_speed,
            "raw_yaw_rate_degps": raw_yaw_rate,
            "continuous_step_xy_m": continuous_step,
            "continuous_speed_xy_mps": continuous_speed,
            "continuous_yaw_rate_degps": continuous_yaw_rate,
            "filtered_step_xy_m": filtered_step,
            "filtered_speed_xy_mps": filtered_speed,
            "filtered_yaw_rate_degps": filtered_yaw_rate,
            "beta_step_xy_m": beta_step,
            "beta_speed_xy_mps": beta_speed,
            "beta_yaw_rate_degps": beta_yaw_rate,
            "stable_step_xy_m": stable_step,
            "stable_speed_xy_mps": stable_speed,
            "stable_yaw_rate_degps": stable_yaw_rate,
            "motion_factor": motion_factor,
            "is_stationary": 1.0 if is_stationary else 0.0,
            "raw_window_x_p2p_m": raw_x_p2p,
            "raw_window_y_p2p_m": raw_y_p2p,
            "raw_window_xy_p2p_m": raw_xy_p2p,
            "filtered_window_x_p2p_m": filtered_x_p2p,
            "filtered_window_y_p2p_m": filtered_y_p2p,
            "filtered_window_xy_p2p_m": filtered_xy_p2p,
            "beta_window_x_p2p_m": beta_x_p2p,
            "beta_window_y_p2p_m": beta_y_p2p,
            "beta_window_xy_p2p_m": beta_xy_p2p,
            "stable_window_x_p2p_m": stable_x_p2p,
            "stable_window_y_p2p_m": stable_y_p2p,
            "stable_window_xy_p2p_m": stable_xy_p2p,
            "stationary_raw_window_x_p2p_m": stationary_raw_x_p2p,
            "stationary_raw_window_y_p2p_m": stationary_raw_y_p2p,
            "stationary_raw_window_xy_p2p_m": stationary_raw_xy_p2p,
            "stationary_filtered_window_x_p2p_m": stationary_filtered_x_p2p,
            "stationary_filtered_window_y_p2p_m": stationary_filtered_y_p2p,
            "stationary_filtered_window_xy_p2p_m": stationary_filtered_xy_p2p,
            "spike_candidate": 1.0 if spike_candidate else 0.0,
            "sustained_motion": 1.0 if sustained_motion else 0.0,
            "static_context_raw_xy_p2p_m": static_context_raw_xy_p2p,
            "static_context_filtered_xy_p2p_m": static_context_filtered_xy_p2p,
            "map_quality_accepted": accepted,
            "map_quality_converged": converged,
            "map_quality_fitness": fitness,
            "map_quality_jump_xy_m": quality_jump,
            "map_quality_scan_points": scan_points,
            "map_quality_map_points": map_points,
            "map_quality_reason_code": reason_code,
            "map_quality_align_ms": align_ms,
            "map_quality_raw_scan_points": raw_scan_points,
            "map_quality_scan_age_sec": quality_scan_age,
            "map_quality_odom_age_sec": quality_odom_age,
            "map_quality_output_jump_yaw_deg": output_jump_yaw,
            "map_quality_map_to_odom_delta_xy_m": map_to_odom_delta_xy,
            "map_quality_map_to_odom_delta_yaw_deg": map_to_odom_delta_yaw,
            "map_quality_overlap_ratio": overlap_ratio,
            "map_quality_scan_odom_delta_sec": scan_odom_delta,
            "map_quality_scan_reused": scan_reused,
            "map_quality_odom_buffer_size": odom_buffer_size,
            "map_quality_localize_busy": localize_busy,
            "map_quality_preprocess_ms": preprocess_ms,
            "map_quality_total_ms": quality_total_ms,
            "filter_status_accepted": filter_accepted,
            "filter_status_raw_step_xy_m": filter_raw_step,
            "filter_status_speed_xy_mps": filter_speed,
            "filter_status_alpha": filter_alpha,
            "filter_status_reject_count": filter_reject_count,
            "filter_status_consecutive_rejects": filter_consecutive_rejects,
            "filter_status_recovered": filter_recovered,
            "filter_status_motion_factor": filter_motion_factor,
            "filter_status_yaw_rate_degps": filter_yaw_rate,
            "filter_status_reason_code": filter_reason_code,
            "filter_status_hold_state_code": filter_hold_state,
            "filter_status_confirm_count": filter_confirm_count,
            "beta_filter_status_accepted": beta_filter_accepted,
            "beta_filter_status_raw_step_xy_m": beta_filter_raw_step,
            "beta_filter_status_speed_xy_mps": beta_filter_speed,
            "beta_filter_status_alpha": beta_filter_alpha,
            "beta_filter_status_reject_count": beta_filter_reject_count,
            "beta_filter_status_consecutive_rejects": beta_filter_consecutive_rejects,
            "beta_filter_status_recovered": beta_filter_recovered,
            "beta_filter_status_motion_factor": beta_filter_motion_factor,
            "beta_filter_status_yaw_rate_degps": beta_filter_yaw_rate,
            "beta_filter_status_reason_code": beta_filter_reason_code,
            "beta_filter_status_hold_state_code": beta_filter_hold_state,
            "beta_filter_status_confirm_count": beta_filter_confirm_count,
            "stable_filter_status_accepted": stable_filter_accepted,
            "stable_filter_status_raw_step_xy_m": stable_filter_raw_step,
            "stable_filter_status_speed_xy_mps": stable_filter_speed,
            "stable_filter_status_alpha": stable_filter_alpha,
            "stable_filter_status_reject_count": stable_filter_reject_count,
            "stable_filter_status_consecutive_rejects": stable_filter_consecutive_rejects,
            "stable_filter_status_recovered": stable_filter_recovered,
            "stable_filter_status_motion_factor": stable_filter_motion_factor,
            "stable_filter_status_yaw_rate_degps": stable_filter_yaw_rate,
            "stable_filter_status_reason_code": stable_filter_reason_code,
            "stable_filter_status_hold_state_code": stable_filter_hold_state,
            "stable_filter_status_confirm_count": stable_filter_confirm_count,
            "continuous_status_ready": continuous_ready,
            "continuous_status_map_update_accepted": continuous_map_update_accepted,
            "continuous_status_correction_alpha": continuous_correction_alpha,
            "continuous_status_correction_age_sec": continuous_correction_age,
            "continuous_status_odom_age_sec": continuous_odom_age,
            "continuous_status_stamp_delta_sec": continuous_stamp_delta,
            "continuous_status_candidate_delta_xy_m": continuous_candidate_delta_xy,
            "continuous_status_candidate_delta_yaw_deg": continuous_candidate_delta_yaw,
            "continuous_status_quality_fitness": continuous_quality_fitness,
            "continuous_status_update_count": continuous_update_count,
            "continuous_status_rejected_count": continuous_rejected_count,
            "continuous_status_match_mode_code": continuous_match_mode_code,
            "continuous_status_reason_code": continuous_reason_code,
            "yaw_fusion_enabled": yaw_fusion_enabled,
            "yaw_fusion_ready": yaw_fusion_ready,
            "yaw_fusion_source_code": yaw_fusion_source_code,
            "yaw_fusion_fast_age_sec": yaw_fusion_fast_age,
            "yaw_fusion_map_age_sec": yaw_fusion_map_age,
            "yaw_fusion_map_ref_yaw_deg": math.degrees(yaw_fusion_map_ref_yaw)
            if math.isfinite(yaw_fusion_map_ref_yaw)
            else math.nan,
            "yaw_fusion_fast_yaw_deg": math.degrees(yaw_fusion_fast_yaw)
            if math.isfinite(yaw_fusion_fast_yaw)
            else math.nan,
            "yaw_fusion_fused_yaw_deg": math.degrees(yaw_fusion_fused_yaw)
            if math.isfinite(yaw_fusion_fused_yaw)
            else math.nan,
            "yaw_fusion_delta_yaw_deg": math.degrees(yaw_fusion_delta_yaw)
            if math.isfinite(yaw_fusion_delta_yaw)
            else math.nan,
            "yaw_fusion_map_correction_alpha": yaw_fusion_map_correction_alpha,
            "raw_age_sec": raw_age,
            "continuous_age_sec": continuous_age,
            "filtered_age_sec": filtered_age,
            "beta_age_sec": beta_age,
            "stable_age_sec": stable_age,
            "quality_age_sec": quality_age,
            "filter_status_age_sec": filter_status_age,
            "beta_filter_status_age_sec": beta_filter_status_age,
            "stable_filter_status_age_sec": stable_filter_status_age,
            "continuous_status_age_sec": continuous_status_age,
            "yaw_fusion_status_age_sec": yaw_fusion_status_age,
        }

        self.rows.append(row)
        self.event_buffer.append(row)
        cutoff_elapsed = (now - self.start_time) - self.args.event_pre_sec - self.args.event_post_sec - 1.0
        while self.event_buffer and float(self.event_buffer[0].get("elapsed_sec", 0.0)) < cutoff_elapsed:
            self.event_buffer.popleft()
        self.writer.writerow(row)
        self.csv_file.flush()
        self.sync_csv()
        self.close_finished_events(now)

        self.previous_raw_pose = self.raw_pose
        self.previous_continuous_pose = self.continuous_pose
        self.previous_filtered_pose = self.filtered_pose
        self.previous_beta_pose = self.beta_pose
        self.previous_stable_pose = self.stable_pose

        elapsed = now - self.start_time
        if (
            self.args.summary_interval > 0.0
            and len(self.rows) > 1
            and elapsed - self.last_summary_elapsed >= self.args.summary_interval
        ):
            self.last_summary_elapsed = elapsed
            self.write_summary()

    def write_summary(self) -> None:
        numeric_rows: list[dict[str, float]] = []
        for row in self.rows:
            numeric_row = {
                key: float(value)
                for key, value in row.items()
                if key != "time_iso" and isinstance(value, (int, float))
            }
            numeric_rows.append(numeric_row)

        if not numeric_rows:
            summary: dict[str, Any] = {"sample_count": 0}
        else:
            temps = [row["max_temp_c"] for row in numeric_rows]
            loads = [row["load1"] for row in numeric_rows]
            raw_p2p = [row["raw_window_xy_p2p_m"] for row in numeric_rows]
            filtered_p2p = [row["filtered_window_xy_p2p_m"] for row in numeric_rows]
            beta_p2p = [row["beta_window_xy_p2p_m"] for row in numeric_rows]
            stable_p2p = [row["stable_window_xy_p2p_m"] for row in numeric_rows]
            stationary_filtered_p2p = [
                row["stationary_filtered_window_xy_p2p_m"] for row in numeric_rows
            ]
            static_context_raw_p2p = [
                row["static_context_raw_xy_p2p_m"] for row in numeric_rows
            ]
            static_context_filtered_p2p = [
                row["static_context_filtered_xy_p2p_m"] for row in numeric_rows
            ]
            quality_jump = [row["map_quality_jump_xy_m"] for row in numeric_rows]
            summary = {
                "sample_count": len(numeric_rows),
                "duration_sec": numeric_rows[-1]["elapsed_sec"],
                "csv_path": str(self.csv_path),
                "event_path": str(self.event_path),
                "event_count": self.event_count,
                "temperature_c": series_stats(numeric_rows, "max_temp_c"),
                "load1": series_stats(numeric_rows, "load1"),
                "load5": series_stats(numeric_rows, "load5"),
                "cpu_freq_avg_mhz": series_stats(numeric_rows, "cpu_freq_avg_mhz"),
                "motion_factor": series_stats(numeric_rows, "motion_factor"),
                "is_stationary": series_stats(numeric_rows, "is_stationary"),
                "raw_window_xy_p2p_m": series_stats(numeric_rows, "raw_window_xy_p2p_m"),
                "filtered_window_xy_p2p_m": series_stats(
                    numeric_rows,
                    "filtered_window_xy_p2p_m",
                ),
                "beta_window_xy_p2p_m": series_stats(
                    numeric_rows,
                    "beta_window_xy_p2p_m",
                ),
                "stable_window_xy_p2p_m": series_stats(
                    numeric_rows,
                    "stable_window_xy_p2p_m",
                ),
                "stationary_filtered_window_xy_p2p_m": series_stats(
                    numeric_rows,
                    "stationary_filtered_window_xy_p2p_m",
                ),
                "static_context_raw_xy_p2p_m": series_stats(
                    numeric_rows,
                    "static_context_raw_xy_p2p_m",
                ),
                "static_context_filtered_xy_p2p_m": series_stats(
                    numeric_rows,
                    "static_context_filtered_xy_p2p_m",
                ),
                "spike_candidate": series_stats(numeric_rows, "spike_candidate"),
                "sustained_motion": series_stats(numeric_rows, "sustained_motion"),
                "raw_filtered_delta_xy_m": series_stats(
                    numeric_rows,
                    "raw_filtered_delta_xy_m",
                ),
                "raw_beta_delta_xy_m": series_stats(
                    numeric_rows,
                    "raw_beta_delta_xy_m",
                ),
                "filtered_beta_delta_xy_m": series_stats(
                    numeric_rows,
                    "filtered_beta_delta_xy_m",
                ),
                "raw_stable_delta_xy_m": series_stats(
                    numeric_rows,
                    "raw_stable_delta_xy_m",
                ),
                "filtered_stable_delta_xy_m": series_stats(
                    numeric_rows,
                    "filtered_stable_delta_xy_m",
                ),
                "map_quality_fitness": series_stats(numeric_rows, "map_quality_fitness"),
                "map_quality_jump_xy_m": series_stats(numeric_rows, "map_quality_jump_xy_m"),
                "map_quality_align_ms": series_stats(numeric_rows, "map_quality_align_ms"),
                "map_quality_overlap_ratio": series_stats(
                    numeric_rows,
                    "map_quality_overlap_ratio",
                ),
                "map_quality_scan_odom_delta_sec": series_stats(
                    numeric_rows,
                    "map_quality_scan_odom_delta_sec",
                ),
                "map_quality_preprocess_ms": series_stats(
                    numeric_rows,
                    "map_quality_preprocess_ms",
                ),
                "map_quality_total_ms": series_stats(
                    numeric_rows,
                    "map_quality_total_ms",
                ),
                "filter_status_alpha": series_stats(numeric_rows, "filter_status_alpha"),
                "filter_status_motion_factor": series_stats(
                    numeric_rows,
                    "filter_status_motion_factor",
                ),
                "beta_filter_status_alpha": series_stats(
                    numeric_rows,
                    "beta_filter_status_alpha",
                ),
                "beta_filter_status_motion_factor": series_stats(
                    numeric_rows,
                    "beta_filter_status_motion_factor",
                ),
                "stable_filter_status_alpha": series_stats(
                    numeric_rows,
                    "stable_filter_status_alpha",
                ),
                "stable_filter_status_motion_factor": series_stats(
                    numeric_rows,
                    "stable_filter_status_motion_factor",
                ),
                "yaw_fusion_ready": series_stats(numeric_rows, "yaw_fusion_ready"),
                "yaw_fusion_fast_age_sec": series_stats(
                    numeric_rows,
                    "yaw_fusion_fast_age_sec",
                ),
                "yaw_fusion_map_age_sec": series_stats(
                    numeric_rows,
                    "yaw_fusion_map_age_sec",
                ),
                "yaw_fusion_delta_yaw_deg": series_stats(
                    numeric_rows,
                    "yaw_fusion_delta_yaw_deg",
                ),
                "correlation": {
                    "temp_vs_raw_window_xy_p2p": pearson(temps, raw_p2p),
                    "temp_vs_filtered_window_xy_p2p": pearson(temps, filtered_p2p),
                    "temp_vs_beta_window_xy_p2p": pearson(temps, beta_p2p),
                    "temp_vs_stable_window_xy_p2p": pearson(temps, stable_p2p),
                    "temp_vs_stationary_filtered_window_xy_p2p": pearson(
                        temps,
                        stationary_filtered_p2p,
                    ),
                    "temp_vs_map_quality_jump_xy": pearson(temps, quality_jump),
                    "load1_vs_raw_window_xy_p2p": pearson(loads, raw_p2p),
                    "load1_vs_filtered_window_xy_p2p": pearson(loads, filtered_p2p),
                    "load1_vs_beta_window_xy_p2p": pearson(loads, beta_p2p),
                    "load1_vs_stable_window_xy_p2p": pearson(loads, stable_p2p),
                    "load1_vs_stationary_filtered_window_xy_p2p": pearson(
                        loads,
                        stationary_filtered_p2p,
                    ),
                    "temp_vs_static_context_raw_xy_p2p": pearson(
                        temps,
                        static_context_raw_p2p,
                    ),
                    "temp_vs_static_context_filtered_xy_p2p": pearson(
                        temps,
                        static_context_filtered_p2p,
                    ),
                    "load1_vs_static_context_raw_xy_p2p": pearson(
                        loads,
                        static_context_raw_p2p,
                    ),
                    "load1_vs_static_context_filtered_xy_p2p": pearson(
                        loads,
                        static_context_filtered_p2p,
                    ),
                    "load1_vs_map_quality_jump_xy": pearson(loads, quality_jump),
                },
            }

        atomic_write_json(self.summary_path, summary)

    def close(self, reason: str = "normal") -> None:
        try:
            self.close_finished_events(time.monotonic() + self.args.event_post_sec + 1.0)
            self.write_summary()
            self.csv_file.flush()
            self.sync_csv(force=True)
            self.event_file.flush()
            os.fsync(self.event_file.fileno())
            if self.event_context_file:
                self.event_context_file.flush()
                os.fsync(self.event_context_file.fileno())
            self.write_stop_marker(reason)
        finally:
            self.csv_file.close()
            self.event_file.close()
            if self.event_context_file:
                self.event_context_file.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record temperature/load/frequency and robot pose health."
    )
    parser.add_argument("--output-dir", default=os.environ.get("LOG_DIR", "."))
    parser.add_argument("--csv-path", default="")
    parser.add_argument("--summary-path", default="")
    parser.add_argument("--event-path", default="")
    parser.add_argument("--event-context-path", default="")
    parser.add_argument("--event-context-csv", action="store_true")
    parser.add_argument("--run-marker-path", default="")
    parser.add_argument("--stop-marker-path", default="")
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--window", type=float, default=10.0)
    parser.add_argument("--summary-interval", type=float, default=30.0)
    parser.add_argument("--fsync-interval", type=float, default=5.0)
    parser.add_argument("--raw-topic", default="/robot/pose")
    parser.add_argument("--continuous-topic", default="/robot/pose_continuous")
    parser.add_argument("--filtered-topic", default="")
    parser.add_argument("--beta-topic", default="")
    parser.add_argument("--stable-topic", default="")
    parser.add_argument("--quality-topic", default="/map_lidar/quality")
    parser.add_argument("--filter-status-topic", default="")
    parser.add_argument("--beta-filter-status-topic", default="")
    parser.add_argument("--stable-filter-status-topic", default="")
    parser.add_argument("--continuous-status-topic", default="/robot/pose_continuous/status")
    parser.add_argument("--yaw-fusion-status-topic", default="")
    parser.add_argument("--static-speed", type=float, default=0.015)
    parser.add_argument("--full-speed", type=float, default=0.12)
    parser.add_argument("--static-yaw-rate-deg", type=float, default=5.0)
    parser.add_argument("--full-yaw-rate-deg", type=float, default=35.0)
    parser.add_argument("--stationary-factor", type=float, default=0.25)
    parser.add_argument("--max-pose-age", type=float, default=2.0)
    parser.add_argument("--event-pre-sec", type=float, default=3.0)
    parser.add_argument("--event-post-sec", type=float, default=3.0)
    parser.add_argument("--event-cooldown-sec", type=float, default=2.0)
    parser.add_argument("--spike-raw-step", type=float, default=0.02)
    parser.add_argument("--spike-filtered-step", type=float, default=0.015)
    parser.add_argument("--spike-map-jump", type=float, default=0.02)
    parser.add_argument("--spike-fitness", type=float, default=0.02)
    parser.add_argument("--sustained-motion-sec", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ThermalPoseMonitor(args)

    stop = {"requested": False, "reason": "normal"}

    def request_stop(signum, _frame):
        stop["requested"] = True
        stop["reason"] = f"signal_{signum}"
        node.get_logger().info(f"signal {signum} received; writing summary and exiting")

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    try:
        while rclpy.ok() and not stop["requested"]:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.close(stop["reason"])
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
