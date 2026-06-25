import math
import statistics
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray

from .pose_math import quat_from_rpy, rpy_from_quat


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def blend_angle(previous: float, current: float, alpha: float) -> float:
    return wrap_angle(previous + alpha * wrap_angle(current - previous))


def bool_param(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def finite_or_nan(values: list[float], index: int) -> float:
    if index >= len(values):
        return math.nan
    value = float(values[index])
    return value if math.isfinite(value) else math.nan


def replace_pose_yaw(
    pose: tuple[float, float, float, float, float, float],
    yaw: float,
) -> tuple[float, float, float, float, float, float]:
    return (
        pose[0],
        pose[1],
        pose[2],
        pose[3],
        pose[4],
        wrap_angle(float(yaw)),
    )


REASON_CODES = {
    "initial": 0.0,
    "accepted": 1.0,
    "stationary_spike": 2.0,
    "hard_reject": 3.0,
    "recovered": 4.0,
    "beta_hold": 5.0,
    "beta_spike_rejected": 6.0,
    "beta_confirmed_motion": 7.0,
    "stable_hold": 8.0,
    "stable_spike_rejected": 9.0,
    "stable_confirmed_motion": 10.0,
    "quality_gate": 11.0,
}

HOLD_STATE_CODES = {
    "idle": 0.0,
    "holding": 1.0,
    "rejected": 2.0,
    "released": 3.0,
}


@dataclass
class FilterResult:
    pose: tuple[float, float, float, float, float, float]
    accepted: bool
    reason: str
    motion_factor: float
    spike_candidate: bool
    raw_step_m: float
    speed_xy_mps: float
    yaw_rate_radps: float
    alpha: float
    recovered: bool = False
    recoverable: bool = False
    hold_state: str = "idle"
    confirm_count: int = 0


class PoseFilterState:
    """r2-style normal/beta pose filter with current Float32 status compatibility."""

    def __init__(
        self,
        static_alpha: float = 0.12,
        moving_alpha: float = 0.92,
        yaw_alpha: float = 0.85,
        static_speed_threshold: float = 0.03,
        full_speed_threshold: float = 0.20,
        static_yaw_rate_rad: float = math.radians(8.0),
        full_yaw_rate_rad: float = math.radians(45.0),
        max_xy_jump: float = 0.50,
        max_xy_speed: float = 5.0,
        max_yaw_jump_rad: float = math.radians(90.0),
        spike_step_m: float = 0.05,
        filter_mode: str = "normal",
        beta_confirm_samples: int = 3,
        beta_return_tolerance_m: float = 0.0,
        stable_confirm_samples: int = 3,
        stable_return_tolerance_m: float = 0.0,
        stable_release_distance_m: float = 0.0,
        stable_window_size: int = 9,
        stable_moving_hold_sec: float = 0.8,
    ) -> None:
        self.static_alpha = float(static_alpha)
        self.moving_alpha = float(moving_alpha)
        self.yaw_alpha = min(max(float(yaw_alpha), 0.0), 1.0)
        self.static_speed_threshold = float(static_speed_threshold)
        self.full_speed_threshold = float(full_speed_threshold)
        self.static_yaw_rate_rad = float(static_yaw_rate_rad)
        self.full_yaw_rate_rad = float(full_yaw_rate_rad)
        self.max_xy_jump = float(max_xy_jump)
        self.max_xy_speed = float(max_xy_speed)
        self.max_yaw_jump_rad = float(max_yaw_jump_rad)
        self.spike_step_m = float(spike_step_m)
        self.filter_mode = str(filter_mode).lower()
        if self.filter_mode not in {"normal", "beta", "stable"}:
            raise ValueError("filter_mode must be normal, beta, or stable")
        self.beta_confirm_samples = max(1, int(beta_confirm_samples))
        if beta_return_tolerance_m > 0.0:
            self.beta_return_tolerance_m = float(beta_return_tolerance_m)
        else:
            self.beta_return_tolerance_m = max(0.003, self.spike_step_m * 0.5)
        self.stable_confirm_samples = max(1, int(stable_confirm_samples))
        if stable_return_tolerance_m > 0.0:
            self.stable_return_tolerance_m = float(stable_return_tolerance_m)
        else:
            self.stable_return_tolerance_m = max(0.003, self.spike_step_m * 0.5)
        if stable_release_distance_m > 0.0:
            self.stable_release_distance_m = float(stable_release_distance_m)
        else:
            self.stable_release_distance_m = max(0.03, self.spike_step_m * 2.5)
        self.stable_window_size = max(3, int(stable_window_size))
        self.stable_moving_hold_sec = max(0.0, float(stable_moving_hold_sec))

        self.filtered: Optional[tuple[float, float, float, float, float, float]] = None
        self.last_raw: Optional[tuple[float, float, float, float, float, float]] = None
        self.last_stamp_sec: Optional[float] = None
        self.last_motion_factor = 0.0

        self.reject_count = 0
        self.accept_count = 0
        self.consecutive_rejects = 0

        self.beta_hold_origin: Optional[
            tuple[float, float, float, float, float, float]
        ] = None
        self.beta_hold_candidate: Optional[
            tuple[float, float, float, float, float, float]
        ] = None
        self.beta_hold_direction: Optional[tuple[float, float]] = None
        self.beta_hold_count = 0

        self.stable_anchor: Optional[
            tuple[float, float, float, float, float, float]
        ] = None
        self.stable_samples: Deque[
            tuple[float, float, float, float, float, float]
        ] = deque(maxlen=self.stable_window_size)
        self.stable_moving_until_sec: Optional[float] = None

    def update(
        self,
        raw: tuple[float, float, float, float, float, float],
        stamp_sec: float,
    ) -> FilterResult:
        raw = (
            float(raw[0]),
            float(raw[1]),
            float(raw[2]),
            float(raw[3]),
            float(raw[4]),
            wrap_angle(float(raw[5])),
        )
        stamp_sec = float(stamp_sec)
        if self.filtered is None or self.last_raw is None:
            self.filtered = raw
            self.last_raw = raw
            self.last_stamp_sec = stamp_sec
            self.stable_anchor = raw
            self.stable_samples.append(raw)
            self.accept_count += 1
            return FilterResult(raw, True, "initial", 0.0, False, 0.0, 0.0, 0.0, 1.0)

        dt = max(stamp_sec - float(self.last_stamp_sec or stamp_sec), 1e-3)
        xy_step = math.hypot(raw[0] - self.last_raw[0], raw[1] - self.last_raw[1])
        speed_xy = xy_step / dt
        yaw_step = abs(wrap_angle(raw[5] - self.last_raw[5]))
        yaw_rate = yaw_step / dt
        motion_factor = self.motion_factor(speed_xy, yaw_rate)

        hard_reject = (
            (xy_step > self.max_xy_jump and speed_xy > self.max_xy_speed)
            or yaw_step > self.max_yaw_jump_rad
        )
        if hard_reject:
            self.reject_count += 1
            self.consecutive_rejects += 1
            return FilterResult(
                self.filtered,
                False,
                "hard_reject",
                motion_factor,
                True,
                xy_step,
                speed_xy,
                yaw_rate,
                0.0,
                recoverable=True,
            )

        if self.filter_mode == "beta":
            return self._update_beta(raw, stamp_sec, motion_factor, xy_step, speed_xy, yaw_rate)

        if self.filter_mode == "stable":
            return self._update_stable(
                raw,
                stamp_sec,
                motion_factor,
                xy_step,
                speed_xy,
                yaw_rate,
            )

        return self._update_normal(raw, stamp_sec, motion_factor, xy_step, speed_xy, yaw_rate)

    def recover(
        self,
        raw: tuple[float, float, float, float, float, float],
        stamp_sec: float,
        raw_step_m: float,
        speed_xy_mps: float,
        yaw_rate_radps: float,
        motion_factor: float,
    ) -> FilterResult:
        raw = (
            float(raw[0]),
            float(raw[1]),
            float(raw[2]),
            float(raw[3]),
            float(raw[4]),
            wrap_angle(float(raw[5])),
        )
        self._clear_beta_hold()
        self._clear_stable_hold()
        self.filtered = raw
        self.last_raw = raw
        self.last_stamp_sec = float(stamp_sec)
        self.stable_anchor = raw
        self.stable_samples.clear()
        self.stable_samples.append(raw)
        self.stable_moving_until_sec = None
        self.accept_count += 1
        self.consecutive_rejects = 0
        self.last_motion_factor = motion_factor
        return FilterResult(
            raw,
            True,
            "recovered",
            motion_factor,
            False,
            raw_step_m,
            speed_xy_mps,
            yaw_rate_radps,
            1.0,
            recovered=True,
        )

    def _update_normal(
        self,
        raw: tuple[float, float, float, float, float, float],
        stamp_sec: float,
        motion_factor: float,
        xy_step: float,
        speed_xy: float,
        yaw_rate: float,
    ) -> FilterResult:
        spike_candidate = (
            self.last_motion_factor < 0.5
            and xy_step > self.spike_step_m
            and speed_xy > self.static_speed_threshold
        )
        self.last_raw = raw
        self.last_stamp_sec = float(stamp_sec)
        if spike_candidate:
            self.reject_count += 1
            self.consecutive_rejects += 1
            return FilterResult(
                self.filtered,
                False,
                "stationary_spike",
                motion_factor,
                True,
                xy_step,
                speed_xy,
                yaw_rate,
                0.0,
            )
        return self._accept_raw(raw, motion_factor, xy_step, speed_xy, yaw_rate, "accepted")

    def _update_beta(
        self,
        raw: tuple[float, float, float, float, float, float],
        stamp_sec: float,
        motion_factor: float,
        xy_step: float,
        speed_xy: float,
        yaw_rate: float,
    ) -> FilterResult:
        raw_to_filtered = math.hypot(raw[0] - self.filtered[0], raw[1] - self.filtered[1])
        if self.beta_hold_origin is not None:
            return self._update_beta_hold(
                raw,
                stamp_sec,
                motion_factor,
                xy_step,
                speed_xy,
                yaw_rate,
            )

        beta_candidate = self.last_motion_factor < 0.5 and raw_to_filtered > self.spike_step_m
        if beta_candidate:
            self._start_beta_hold(raw)
            self.last_raw = raw
            self.last_stamp_sec = float(stamp_sec)
            self.reject_count += 1
            self.consecutive_rejects += 1
            return FilterResult(
                self.filtered,
                False,
                "beta_hold",
                motion_factor,
                True,
                xy_step,
                speed_xy,
                yaw_rate,
                0.0,
                hold_state="holding",
                confirm_count=self.beta_hold_count,
            )

        self.last_raw = raw
        self.last_stamp_sec = float(stamp_sec)
        return self._accept_raw(raw, motion_factor, xy_step, speed_xy, yaw_rate, "accepted")

    def _start_beta_hold(
        self,
        raw: tuple[float, float, float, float, float, float],
    ) -> None:
        self.beta_hold_origin = self.filtered
        self.beta_hold_candidate = raw
        dx = raw[0] - self.filtered[0]
        dy = raw[1] - self.filtered[1]
        norm = max(math.hypot(dx, dy), 1e-9)
        self.beta_hold_direction = (dx / norm, dy / norm)
        self.beta_hold_count = 1

    def _clear_beta_hold(self) -> None:
        self.beta_hold_origin = None
        self.beta_hold_candidate = None
        self.beta_hold_direction = None
        self.beta_hold_count = 0

    def _update_beta_hold(
        self,
        raw: tuple[float, float, float, float, float, float],
        stamp_sec: float,
        motion_factor: float,
        xy_step: float,
        speed_xy: float,
        yaw_rate: float,
    ) -> FilterResult:
        origin = self.beta_hold_origin
        direction = self.beta_hold_direction
        candidate = self.beta_hold_candidate
        origin_to_raw = math.hypot(raw[0] - origin[0], raw[1] - origin[1])

        self.last_raw = raw
        self.last_stamp_sec = float(stamp_sec)

        if origin_to_raw <= self.beta_return_tolerance_m:
            self._clear_beta_hold()
            self.reject_count += 1
            self.consecutive_rejects += 1
            return FilterResult(
                self.filtered,
                False,
                "beta_spike_rejected",
                motion_factor,
                True,
                xy_step,
                speed_xy,
                yaw_rate,
                0.0,
                hold_state="rejected",
            )

        vx = raw[0] - origin[0]
        vy = raw[1] - origin[1]
        progress = vx * direction[0] + vy * direction[1]
        lateral = abs(vx * direction[1] - vy * direction[0])
        candidate_progress = (
            (candidate[0] - origin[0]) * direction[0]
            + (candidate[1] - origin[1]) * direction[1]
        )
        continuing = (
            progress >= max(candidate_progress, self.spike_step_m)
            and lateral <= max(self.spike_step_m, origin_to_raw * 0.35)
        )
        if continuing:
            self.beta_hold_count += 1

        if self.beta_hold_count >= self.beta_confirm_samples:
            confirm_count = self.beta_hold_count
            self._clear_beta_hold()
            return self._accept_raw(
                raw,
                max(motion_factor, 1.0),
                xy_step,
                speed_xy,
                yaw_rate,
                "beta_confirmed_motion",
                hold_state="released",
                confirm_count=confirm_count,
            )

        self.reject_count += 1
        self.consecutive_rejects += 1
        return FilterResult(
            self.filtered,
            False,
            "beta_hold",
            motion_factor,
            True,
            xy_step,
            speed_xy,
            yaw_rate,
            0.0,
            hold_state="holding",
            confirm_count=self.beta_hold_count,
        )

    def _update_stable(
        self,
        raw: tuple[float, float, float, float, float, float],
        stamp_sec: float,
        motion_factor: float,
        xy_step: float,
        speed_xy: float,
        yaw_rate: float,
    ) -> FilterResult:
        if self.stable_anchor is None:
            self.stable_anchor = self.filtered

        if self.beta_hold_origin is not None:
            return self._update_stable_hold(
                raw,
                stamp_sec,
                motion_factor,
                xy_step,
                speed_xy,
                yaw_rate,
            )

        if (
            self.stable_moving_until_sec is not None
            and stamp_sec <= self.stable_moving_until_sec
        ):
            self.last_raw = raw
            self.last_stamp_sec = float(stamp_sec)
            self.stable_anchor = raw
            self.stable_samples.clear()
            self.stable_samples.append(raw)
            if motion_factor > 0.2:
                self.stable_moving_until_sec = stamp_sec + self.stable_moving_hold_sec
            return self._accept_raw(
                raw,
                max(motion_factor, 0.75),
                xy_step,
                speed_xy,
                yaw_rate,
                "accepted",
            )

        anchor = self.stable_anchor or self.filtered
        raw_to_anchor = math.hypot(raw[0] - anchor[0], raw[1] - anchor[1])
        if raw_to_anchor > self.spike_step_m:
            self._start_beta_hold(raw)
            self.last_raw = raw
            self.last_stamp_sec = float(stamp_sec)
            self.reject_count += 1
            self.consecutive_rejects += 1
            return FilterResult(
                self.filtered,
                False,
                "stable_hold",
                motion_factor,
                True,
                xy_step,
                speed_xy,
                yaw_rate,
                0.0,
                hold_state="holding",
                confirm_count=self.beta_hold_count,
            )

        self.last_raw = raw
        self.last_stamp_sec = float(stamp_sec)
        self._remember_stable_sample(raw)
        target = self.stable_anchor or raw
        return self._accept_raw(
            target,
            0.0,
            xy_step,
            speed_xy,
            yaw_rate,
            "accepted",
        )

    def _update_stable_hold(
        self,
        raw: tuple[float, float, float, float, float, float],
        stamp_sec: float,
        motion_factor: float,
        xy_step: float,
        speed_xy: float,
        yaw_rate: float,
    ) -> FilterResult:
        origin = self.beta_hold_origin
        direction = self.beta_hold_direction
        candidate = self.beta_hold_candidate
        if origin is None or direction is None or candidate is None:
            self._clear_stable_hold()
            return self._update_stable(raw, stamp_sec, motion_factor, xy_step, speed_xy, yaw_rate)

        origin_to_raw = math.hypot(raw[0] - origin[0], raw[1] - origin[1])
        self.last_raw = raw
        self.last_stamp_sec = float(stamp_sec)

        if origin_to_raw <= self.stable_return_tolerance_m:
            self._clear_stable_hold()
            self._remember_stable_sample(raw)
            self.reject_count += 1
            self.consecutive_rejects += 1
            return FilterResult(
                self.filtered,
                False,
                "stable_spike_rejected",
                motion_factor,
                True,
                xy_step,
                speed_xy,
                yaw_rate,
                0.0,
                hold_state="rejected",
            )

        vx = raw[0] - origin[0]
        vy = raw[1] - origin[1]
        progress = vx * direction[0] + vy * direction[1]
        lateral = abs(vx * direction[1] - vy * direction[0])
        candidate_progress = (
            (candidate[0] - origin[0]) * direction[0]
            + (candidate[1] - origin[1]) * direction[1]
        )
        continuing = (
            progress >= max(candidate_progress, self.spike_step_m)
            and lateral <= max(self.spike_step_m, origin_to_raw * 0.35)
        )
        if continuing:
            self.beta_hold_count += 1
            self.beta_hold_candidate = raw

        if (
            self.beta_hold_count >= self.stable_confirm_samples
            and origin_to_raw >= self.stable_release_distance_m
        ):
            confirm_count = self.beta_hold_count
            self._clear_stable_hold()
            self.stable_moving_until_sec = stamp_sec + self.stable_moving_hold_sec
            self.stable_anchor = raw
            self.stable_samples.clear()
            self.stable_samples.append(raw)
            return self._accept_raw(
                raw,
                max(motion_factor, 1.0),
                xy_step,
                speed_xy,
                yaw_rate,
                "stable_confirmed_motion",
                hold_state="released",
                confirm_count=confirm_count,
            )

        self.reject_count += 1
        self.consecutive_rejects += 1
        return FilterResult(
            self.filtered,
            False,
            "stable_hold",
            motion_factor,
            True,
            xy_step,
            speed_xy,
            yaw_rate,
            0.0,
            hold_state="holding",
            confirm_count=self.beta_hold_count,
        )

    def _clear_stable_hold(self) -> None:
        self._clear_beta_hold()

    def _remember_stable_sample(
        self,
        raw: tuple[float, float, float, float, float, float],
    ) -> None:
        self.stable_samples.append(raw)
        self.stable_anchor = self._stable_window_pose()

    def _stable_window_pose(self) -> tuple[float, float, float, float, float, float]:
        samples = list(self.stable_samples)
        if not samples:
            return self.filtered

        def median_at(index: int) -> float:
            return float(statistics.median(sample[index] for sample in samples))

        sin_yaw = sum(math.sin(sample[5]) for sample in samples)
        cos_yaw = sum(math.cos(sample[5]) for sample in samples)
        yaw = math.atan2(sin_yaw, cos_yaw)
        return (
            median_at(0),
            median_at(1),
            median_at(2),
            median_at(3),
            median_at(4),
            wrap_angle(yaw),
        )

    def _accept_raw(
        self,
        raw: tuple[float, float, float, float, float, float],
        motion_factor: float,
        raw_step_m: float,
        speed_xy_mps: float,
        yaw_rate_radps: float,
        reason: str,
        hold_state: str = "idle",
        confirm_count: int = 0,
    ) -> FilterResult:
        alpha = self.alpha_for_motion(motion_factor)
        filtered = self.blend_pose(self.filtered, raw, alpha)
        self.filtered = filtered
        self.last_motion_factor = motion_factor
        self.accept_count += 1
        self.consecutive_rejects = 0
        return FilterResult(
            filtered,
            True,
            reason,
            motion_factor,
            False,
            raw_step_m,
            speed_xy_mps,
            yaw_rate_radps,
            alpha,
            hold_state=hold_state,
            confirm_count=confirm_count,
        )

    def blend_pose(
        self,
        previous: tuple[float, float, float, float, float, float],
        current: tuple[float, float, float, float, float, float],
        alpha: float,
    ) -> tuple[float, float, float, float, float, float]:
        return (
            self.ema(previous[0], current[0], alpha),
            self.ema(previous[1], current[1], alpha),
            self.ema(previous[2], current[2], self.static_alpha),
            self.ema(previous[3], current[3], self.static_alpha),
            self.ema(previous[4], current[4], self.static_alpha),
            blend_angle(previous[5], current[5], self.yaw_alpha),
        )

    def motion_factor(self, speed_xy: float, yaw_rate: float) -> float:
        speed_factor = self.ramp(
            speed_xy,
            self.static_speed_threshold,
            self.full_speed_threshold,
        )
        yaw_factor = self.ramp(
            yaw_rate,
            self.static_yaw_rate_rad,
            self.full_yaw_rate_rad,
        )
        return max(speed_factor, yaw_factor)

    def alpha_for_motion(self, motion_factor: float) -> float:
        static_alpha = min(max(self.static_alpha, 0.0), 1.0)
        moving_alpha = min(max(self.moving_alpha, 0.0), 1.0)
        factor = min(max(motion_factor, 0.0), 1.0)
        return static_alpha + factor * (moving_alpha - static_alpha)

    @staticmethod
    def ema(previous: float, current: float, alpha: float) -> float:
        return previous + alpha * (current - previous)

    @staticmethod
    def ramp(value: float, low: float, high: float) -> float:
        if high <= low:
            return 1.0 if value >= high else 0.0
        return min(max((value - low) / (high - low), 0.0), 1.0)


class PoseStabilityFilter(Node):
    """Low-latency pose smoother with optional r2 beta spike confirmation mode."""

    def __init__(self) -> None:
        super().__init__("pose_stability_filter")

        self.input_pose_topic = self.declare_parameter(
            "input_pose_topic",
            "/robot/pose",
        ).value
        self.output_pose_topic = self.declare_parameter(
            "output_pose_topic",
            "/robot/pose_filtered",
        ).value
        self.output_pose_alias_topic = self.declare_parameter(
            "output_pose_alias_topic",
            "",
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic",
            "/lidar/pose_filter/status",
        ).value
        self.filter_mode = str(
            self.declare_parameter("filter_mode", "normal").value
        ).lower()
        self.static_alpha = float(self.declare_parameter("static_alpha", 0.12).value)
        self.moving_alpha = float(self.declare_parameter("moving_alpha", 0.92).value)
        self.yaw_alpha = float(self.declare_parameter("yaw_alpha", 0.85).value)
        self.use_raw_yaw = bool_param(
            self.declare_parameter("use_raw_yaw", False).value
        )
        self.static_speed_threshold = float(
            self.declare_parameter("static_speed_threshold", 0.03).value
        )
        self.full_speed_threshold = float(
            self.declare_parameter("full_speed_threshold", 0.20).value
        )
        self.static_yaw_rate_rad = math.radians(
            float(self.declare_parameter("static_yaw_rate_deg", 8.0).value)
        )
        self.full_yaw_rate_rad = math.radians(
            float(self.declare_parameter("full_yaw_rate_deg", 45.0).value)
        )
        self.max_xy_jump = float(self.declare_parameter("max_xy_jump", 0.50).value)
        self.max_xy_speed = float(self.declare_parameter("max_xy_speed", 5.0).value)
        self.max_yaw_jump_rad = math.radians(
            float(self.declare_parameter("max_yaw_jump_deg", 90.0).value)
        )
        self.spike_step_m = float(self.declare_parameter("spike_step_m", 0.05).value)
        self.beta_confirm_samples = int(
            self.declare_parameter("beta_confirm_samples", 3).value
        )
        self.beta_return_tolerance_m = float(
            self.declare_parameter("beta_return_tolerance_m", 0.0).value
        )
        self.stable_confirm_samples = int(
            self.declare_parameter("stable_confirm_samples", 3).value
        )
        self.stable_return_tolerance_m = float(
            self.declare_parameter("stable_return_tolerance_m", 0.0).value
        )
        self.stable_release_distance_m = float(
            self.declare_parameter("stable_release_distance_m", 0.0).value
        )
        self.stable_window_size = int(
            self.declare_parameter("stable_window_size", 9).value
        )
        self.stable_moving_hold_sec = float(
            self.declare_parameter("stable_moving_hold_sec", 0.8).value
        )
        self.recovery_reject_count = int(
            self.declare_parameter("recovery_reject_count", 10).value
        )
        self.recovery_time_sec = float(
            self.declare_parameter("recovery_time_sec", 1.0).value
        )
        self.log_period_sec = float(
            self.declare_parameter("log_period_sec", 1.0).value
        )
        self.output_stamp_mode = str(
            self.declare_parameter("output_stamp_mode", "now").value
        ).lower()
        if self.output_stamp_mode not in {"now", "input"}:
            self.get_logger().warn(
                "output_stamp_mode must be 'now' or 'input'; falling back to 'now'"
            )
            self.output_stamp_mode = "now"
        self.quality_gate_enabled = bool_param(
            self.declare_parameter("quality_gate_enabled", True).value
        )
        self.quality_topic = str(
            self.declare_parameter("quality_topic", "/map_lidar/quality").value
        )
        self.quality_timeout_sec = float(
            self.declare_parameter("quality_timeout_sec", 1.0).value
        )
        self.quality_max_jump_m = float(
            self.declare_parameter("quality_max_jump_m", 0.08).value
        )
        self.quality_max_fitness = float(
            self.declare_parameter("quality_max_fitness", 0.20).value
        )

        self.state = PoseFilterState(
            static_alpha=self.static_alpha,
            moving_alpha=self.moving_alpha,
            yaw_alpha=self.yaw_alpha,
            static_speed_threshold=self.static_speed_threshold,
            full_speed_threshold=self.full_speed_threshold,
            static_yaw_rate_rad=self.static_yaw_rate_rad,
            full_yaw_rate_rad=self.full_yaw_rate_rad,
            max_xy_jump=self.max_xy_jump,
            max_xy_speed=self.max_xy_speed,
            max_yaw_jump_rad=self.max_yaw_jump_rad,
            spike_step_m=self.spike_step_m,
            filter_mode=self.filter_mode,
            beta_confirm_samples=self.beta_confirm_samples,
            beta_return_tolerance_m=self.beta_return_tolerance_m,
            stable_confirm_samples=self.stable_confirm_samples,
            stable_return_tolerance_m=self.stable_return_tolerance_m,
            stable_release_distance_m=self.stable_release_distance_m,
            stable_window_size=self.stable_window_size,
            stable_moving_hold_sec=self.stable_moving_hold_sec,
        )

        self.last_receive_time: Optional[Time] = None
        self.first_reject_time: Optional[Time] = None
        self.last_log_time = self.get_clock().now()
        self.latest_quality_time: Optional[Time] = None
        self.latest_quality: Optional[list[float]] = None

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 20)
        self.pose_alias_pub = (
            self.create_publisher(PoseStamped, self.output_pose_alias_topic, 20)
            if self.output_pose_alias_topic
            else None
        )
        self.status_pub = self.create_publisher(Float32MultiArray, self.status_topic, 20)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.input_pose_topic,
            self.pose_callback,
            20,
        )
        self.quality_sub = None
        if self.quality_gate_enabled and self.quality_topic:
            self.quality_sub = self.create_subscription(
                Float32MultiArray,
                self.quality_topic,
                self.quality_callback,
                20,
            )

        self.get_logger().info(
            f"pose stability filter ready: {self.input_pose_topic} -> {self.output_pose_topic}"
            f"{f' and {self.output_pose_alias_topic}' if self.output_pose_alias_topic else ''} "
            f"mode={self.filter_mode}"
        )
        self.get_logger().info(
            "filter params: "
            f"static_alpha={self.static_alpha:.3f} "
            f"moving_alpha={self.moving_alpha:.3f} "
            f"yaw_alpha={self.state.yaw_alpha:.3f} "
            f"use_raw_yaw={self.use_raw_yaw} "
            f"static_speed={self.static_speed_threshold:.3f} "
            f"full_speed={self.full_speed_threshold:.3f} "
            f"static_yaw_rate_deg={math.degrees(self.static_yaw_rate_rad):.1f} "
            f"full_yaw_rate_deg={math.degrees(self.full_yaw_rate_rad):.1f} "
            f"spike_step={self.spike_step_m:.3f} "
            f"max_xy_jump={self.max_xy_jump:.3f} "
            f"max_xy_speed={self.max_xy_speed:.3f} "
            f"max_yaw_jump_deg={math.degrees(self.max_yaw_jump_rad):.1f} "
            f"beta_confirm_samples={self.beta_confirm_samples} "
            f"beta_return_tolerance={self.state.beta_return_tolerance_m:.3f} "
            f"stable_confirm_samples={self.stable_confirm_samples} "
            f"stable_return_tolerance={self.state.stable_return_tolerance_m:.3f} "
            f"stable_release_distance={self.state.stable_release_distance_m:.3f} "
            f"stable_window_size={self.stable_window_size} "
            f"stable_moving_hold_sec={self.stable_moving_hold_sec:.2f} "
            f"stamp_mode={self.output_stamp_mode} "
            f"quality_gate={self.quality_gate_enabled} "
            f"quality_max_jump={self.quality_max_jump_m:.3f} "
            f"quality_max_fitness={self.quality_max_fitness:.3f}"
        )

    def pose_stamp(self, msg: PoseStamped) -> Time:
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            return self.get_clock().now()
        return stamp

    def quality_callback(self, msg: Float32MultiArray) -> None:
        self.latest_quality_time = self.get_clock().now()
        self.latest_quality = [float(value) for value in msg.data]

    def quality_gate_reason(self, receive_time: Time) -> str:
        if not self.quality_gate_enabled or self.latest_quality is None:
            return ""
        if self.latest_quality_time is None:
            return ""

        age_sec = (receive_time - self.latest_quality_time).nanoseconds / 1e9
        if age_sec > self.quality_timeout_sec:
            return ""

        values = self.latest_quality
        accepted = finite_or_nan(values, 0)
        converged = finite_or_nan(values, 1)
        fitness = finite_or_nan(values, 2)
        jump_xy = finite_or_nan(values, 7)
        reason_code = finite_or_nan(values, 10)

        if math.isfinite(accepted) and accepted < 0.5:
            return "quality_gate"
        if math.isfinite(converged) and converged < 0.5:
            return "quality_gate"
        if (
            self.quality_max_fitness > 0.0
            and math.isfinite(fitness)
            and fitness > self.quality_max_fitness
        ):
            return "quality_gate"
        if (
            self.quality_max_jump_m > 0.0
            and math.isfinite(jump_xy)
            and jump_xy > self.quality_max_jump_m
        ):
            return "quality_gate"
        if math.isfinite(reason_code) and int(reason_code) != 0:
            return "quality_gate"
        return ""

    def quality_gate_result(
        self,
        raw: tuple[float, float, float, float, float, float],
        stamp_sec: float,
        reason: str,
    ) -> Optional[FilterResult]:
        if not reason or self.state.filtered is None or self.state.last_raw is None:
            return None

        dt = max(stamp_sec - float(self.state.last_stamp_sec or stamp_sec), 1e-3)
        xy_step = math.hypot(
            float(raw[0]) - self.state.last_raw[0],
            float(raw[1]) - self.state.last_raw[1],
        )
        speed_xy = xy_step / dt
        yaw_step = abs(wrap_angle(float(raw[5]) - self.state.last_raw[5]))
        yaw_rate = yaw_step / dt
        motion_factor = self.state.motion_factor(speed_xy, yaw_rate)

        self.state.last_raw = (
            float(raw[0]),
            float(raw[1]),
            float(raw[2]),
            float(raw[3]),
            float(raw[4]),
            wrap_angle(float(raw[5])),
        )
        self.state.last_stamp_sec = float(stamp_sec)
        self.state.reject_count += 1
        self.state.consecutive_rejects += 1

        return FilterResult(
            self.state.filtered,
            False,
            reason,
            motion_factor,
            True,
            xy_step,
            speed_xy,
            yaw_rate,
            0.0,
            hold_state="holding",
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        receive_time = self.get_clock().now()
        stamp = self.pose_stamp(msg)
        stamp_sec = stamp.nanoseconds / 1e9
        roll, pitch, yaw = rpy_from_quat(msg.pose.orientation)
        raw = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            roll,
            pitch,
            yaw,
        )

        gate_reason = self.quality_gate_reason(receive_time)
        result = self.quality_gate_result(raw, stamp_sec, gate_reason)
        if result is None:
            result = self.state.update(raw, stamp_sec)
        if result.recoverable:
            if self.first_reject_time is None:
                self.first_reject_time = receive_time
            reject_duration = (receive_time - self.first_reject_time).nanoseconds / 1e9
            if self.should_recover_from_rejects(reject_duration):
                result = self.state.recover(
                    raw,
                    stamp_sec,
                    result.raw_step_m,
                    result.speed_xy_mps,
                    result.yaw_rate_radps,
                    result.motion_factor,
                )
                self.first_reject_time = None
                self.log_recover(result.raw_step_m, result.speed_xy_mps, result.yaw_rate_radps)
            else:
                self.log_reject(
                    result.reason,
                    result.raw_step_m,
                    result.speed_xy_mps,
                    result.yaw_rate_radps,
                    reject_duration,
                )
        else:
            self.first_reject_time = None

        self.apply_raw_yaw_override(result, raw)
        output = self.pose_from_tuple(msg, result.pose)
        self.publish_filtered_pose(output)
        self.publish_status(msg, output, result)
        self.log_result(result)
        self.last_receive_time = receive_time

    def apply_raw_yaw_override(
        self,
        result: FilterResult,
        raw: tuple[float, float, float, float, float, float],
    ) -> None:
        if not self.use_raw_yaw:
            return
        result.pose = replace_pose_yaw(result.pose, raw[5])
        if self.state.filtered is not None:
            self.state.filtered = replace_pose_yaw(self.state.filtered, raw[5])

    def should_recover_from_rejects(self, reject_duration: float) -> bool:
        count_reached = (
            self.recovery_reject_count > 0
            and self.state.consecutive_rejects >= self.recovery_reject_count
        )
        time_reached = (
            self.recovery_time_sec > 0.0
            and reject_duration >= self.recovery_time_sec
        )
        return count_reached or time_reached

    def publish_filtered_pose(self, msg: PoseStamped) -> None:
        self.pose_pub.publish(msg)
        if self.pose_alias_pub:
            self.pose_alias_pub.publish(msg)

    def pose_from_tuple(
        self,
        source: PoseStamped,
        pose: tuple[float, float, float, float, float, float],
    ) -> PoseStamped:
        output = PoseStamped()
        output.header = source.header
        if self.output_stamp_mode == "now":
            output.header.stamp = self.get_clock().now().to_msg()
        output.pose.position.x = pose[0]
        output.pose.position.y = pose[1]
        output.pose.position.z = pose[2]
        output.pose.orientation = quat_from_rpy(pose[3], pose[4], pose[5])
        return output

    def publish_status(
        self,
        raw: PoseStamped,
        filtered: PoseStamped,
        result: FilterResult,
    ) -> None:
        _, _, raw_yaw = rpy_from_quat(raw.pose.orientation)
        _, _, filtered_yaw = rpy_from_quat(filtered.pose.orientation)
        status = Float32MultiArray()
        mode_code = 0.0
        if self.filter_mode == "beta":
            mode_code = 1.0
        elif self.filter_mode == "stable":
            mode_code = 2.0
        status.data = [
            1.0 if result.accepted else 0.0,
            float(raw.pose.position.x),
            float(raw.pose.position.y),
            float(filtered.pose.position.x),
            float(filtered.pose.position.y),
            float(result.raw_step_m),
            float(result.speed_xy_mps),
            float(result.alpha),
            float(raw_yaw),
            float(filtered_yaw),
            float(self.state.reject_count),
            float(self.state.accept_count),
            float(self.state.consecutive_rejects),
            1.0 if result.recovered else 0.0,
            float(result.motion_factor),
            float(math.degrees(result.yaw_rate_radps)),
            mode_code,
            float(REASON_CODES.get(result.reason, -1.0)),
            float(HOLD_STATE_CODES.get(result.hold_state, -1.0)),
            float(result.confirm_count),
            1.0 if result.spike_candidate else 0.0,
            1.0 if result.reason == "quality_gate" else 0.0,
            float(self.quality_age_sec()),
            finite_or_nan(self.latest_quality or [], 2),
            finite_or_nan(self.latest_quality or [], 7),
            float(self.state.yaw_alpha),
            1.0 if self.use_raw_yaw else 0.0,
        ]
        self.status_pub.publish(status)

    def quality_age_sec(self) -> float:
        if self.latest_quality_time is None:
            return math.nan
        return (self.get_clock().now() - self.latest_quality_time).nanoseconds / 1e9

    def log_result(self, result: FilterResult) -> None:
        now = self.get_clock().now()
        should_rate_limit = result.accepted and not result.recovered
        if should_rate_limit and (now - self.last_log_time).nanoseconds / 1e9 < self.log_period_sec:
            return
        self.last_log_time = now
        message = (
            f"pose filter {result.reason}: step={result.raw_step_m:.4f}m "
            f"speed={result.speed_xy_mps:.3f}m/s alpha={result.alpha:.2f} "
            f"yaw_alpha={self.state.yaw_alpha:.2f} "
            f"raw_yaw={1 if self.use_raw_yaw else 0} "
            f"motion={result.motion_factor:.2f} yaw_rate={math.degrees(result.yaw_rate_radps):.1f}deg/s "
            f"hold={result.hold_state} confirm={result.confirm_count} "
            f"rejects={self.state.reject_count}"
        )
        if result.accepted:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)

    def log_reject(
        self,
        reason: str,
        xy_step: float,
        speed_xy: float,
        yaw_rate: float,
        reject_duration: float,
    ) -> None:
        self.get_logger().warn(
            f"pose filter rejected {reason}: xy_step={xy_step:.3f}m "
            f"speed={speed_xy:.3f}m/s yaw_rate={math.degrees(yaw_rate):.1f}deg/s "
            f"rejects={self.state.reject_count} consecutive={self.state.consecutive_rejects} "
            f"duration={reject_duration:.2f}s"
        )

    def log_recover(self, xy_step: float, speed_xy: float, yaw_rate: float) -> None:
        self.get_logger().warn(
            "pose filter recovered by rebasing after persistent hard reject: "
            f"xy_step={xy_step:.3f}m speed={speed_xy:.3f}m/s "
            f"yaw_rate={math.degrees(yaw_rate):.1f}deg/s"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseStabilityFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
