#!/usr/bin/env python3
import math
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

from .pose_math import copy_pose, quat_from_rpy, rpy_from_quat


REASON_OK = 0
REASON_WAIT_INPUT = 1
REASON_WAIT_CORRECTION = 2
REASON_COLLECTING = 3
REASON_UNSTABLE_XY = 4
REASON_UNSTABLE_YAW = 5
REASON_QUALITY_GATE = 6


def wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pose_yaw(pose: Pose) -> float:
    return rpy_from_quat(pose.orientation)[2]


def circular_mean(angles: list[float]) -> float:
    if not angles:
        return 0.0
    sin_sum = sum(math.sin(a) for a in angles)
    cos_sum = sum(math.cos(a) for a in angles)
    return math.atan2(sin_sum, cos_sum)


def circular_std(angles: list[float], mean: float | None = None) -> float:
    if not angles:
        return math.nan
    center = circular_mean(angles) if mean is None else mean
    return math.sqrt(sum(wrap_angle(a - center) ** 2 for a in angles) / len(angles))


def sample_stats(samples: list["PoseSample"]) -> tuple[Pose, float, float, float]:
    count = max(1, len(samples))
    mean_x = sum(s.pose.position.x for s in samples) / count
    mean_y = sum(s.pose.position.y for s in samples) / count
    mean_z = sum(s.pose.position.z for s in samples) / count
    mean_yaw = circular_mean([s.yaw for s in samples])
    xy_std = math.sqrt(
        sum(
            (s.pose.position.x - mean_x) ** 2 + (s.pose.position.y - mean_y) ** 2
            for s in samples
        )
        / count
    )
    yaw_std = circular_std([s.yaw for s in samples], mean_yaw)

    origin = Pose()
    origin.position.x = mean_x
    origin.position.y = mean_y
    origin.position.z = mean_z
    origin.orientation = quat_from_rpy(0.0, 0.0, mean_yaw)
    window_sec = samples[-1].wall_sec - samples[0].wall_sec if len(samples) >= 2 else 0.0
    return origin, xy_std, yaw_std, window_sec


def pose_relative_to_planar_origin(origin: Pose, current: Pose) -> Pose:
    origin_yaw = pose_yaw(origin)
    current_yaw = pose_yaw(current)
    dx = current.position.x - origin.position.x
    dy = current.position.y - origin.position.y
    c = math.cos(origin_yaw)
    s = math.sin(origin_yaw)

    out = Pose()
    out.position.x = c * dx + s * dy
    out.position.y = -s * dx + c * dy
    out.position.z = 0.0
    out.orientation = quat_from_rpy(0.0, 0.0, wrap_angle(current_yaw - origin_yaw))
    return out


@dataclass
class PoseSample:
    wall_sec: float
    stamp_sec: float
    pose: Pose
    yaw: float


@dataclass
class LockDecision:
    locked: bool
    reason: int
    sample_count: int
    window_sec: float
    xy_std: float
    yaw_std: float


class MatchOriginState:
    def __init__(
        self,
        *,
        min_lock_sec: float = 2.5,
        min_samples: int = 20,
        max_xy_std: float = 0.08,
        max_yaw_std_rad: float = math.radians(5.0),
        max_buffer_sec: float | None = None,
    ) -> None:
        self.min_lock_sec = max(0.0, float(min_lock_sec))
        self.min_samples = max(1, int(min_samples))
        self.max_xy_std = max(0.0, float(max_xy_std))
        self.max_yaw_std_rad = max(0.0, float(max_yaw_std_rad))
        self.max_buffer_sec = (
            max(self.min_lock_sec, float(max_buffer_sec))
            if max_buffer_sec is not None
            else max(5.0, self.min_lock_sec * 2.0)
        )
        self.samples: deque[PoseSample] = deque()
        self.origin: Pose | None = None
        self.xy_std = math.nan
        self.yaw_std = math.nan
        self.window_sec = 0.0

    @property
    def locked(self) -> bool:
        return self.origin is not None

    def reset(self) -> None:
        self.samples.clear()
        self.origin = None
        self.xy_std = math.nan
        self.yaw_std = math.nan
        self.window_sec = 0.0

    def add_sample(self, sample: PoseSample) -> LockDecision:
        if self.locked:
            return LockDecision(
                True,
                REASON_OK,
                len(self.samples),
                self.window_sec,
                self.xy_std,
                self.yaw_std,
            )

        self.samples.append(sample)
        self.trim(sample.wall_sec)
        samples = list(self.samples)
        if not samples:
            return LockDecision(False, REASON_WAIT_INPUT, 0, 0.0, math.nan, math.nan)

        origin, xy_std, yaw_std, window_sec = sample_stats(samples)
        self.xy_std = xy_std
        self.yaw_std = yaw_std
        self.window_sec = window_sec

        if len(samples) < self.min_samples or window_sec < self.min_lock_sec:
            return LockDecision(False, REASON_COLLECTING, len(samples), window_sec, xy_std, yaw_std)
        if xy_std > self.max_xy_std:
            return LockDecision(False, REASON_UNSTABLE_XY, len(samples), window_sec, xy_std, yaw_std)
        if yaw_std > self.max_yaw_std_rad:
            return LockDecision(False, REASON_UNSTABLE_YAW, len(samples), window_sec, xy_std, yaw_std)

        self.origin = origin
        return LockDecision(True, REASON_OK, len(samples), window_sec, xy_std, yaw_std)

    def trim(self, now: float) -> None:
        while self.samples and now - self.samples[0].wall_sec > self.max_buffer_sec:
            self.samples.popleft()


class MatchOriginPose(Node):
    def __init__(self) -> None:
        super().__init__("match_origin_pose")

        self.declare_parameter("input_pose_topic", "/robot/pose_continuous")
        self.declare_parameter("input_status_topic", "/robot/pose_continuous/status")
        self.declare_parameter("output_pose_topic", "/robot/pose_match")
        self.declare_parameter("output_odom_topic", "/robot/odom_match")
        self.declare_parameter("status_topic", "/robot/match_origin/status")
        self.declare_parameter("output_frame_id", "match_origin")
        self.declare_parameter("child_frame_id", "robot_base")
        self.declare_parameter("min_lock_sec", 2.5)
        self.declare_parameter("min_samples", 20)
        self.declare_parameter("max_xy_std", 0.08)
        self.declare_parameter("max_yaw_std_deg", 5.0)
        self.declare_parameter("require_correction_ready", True)
        self.declare_parameter("status_timeout_sec", 2.0)
        self.declare_parameter("use_quality_gate", False)
        self.declare_parameter("fitness_warn", 0.30)
        self.declare_parameter("log_period_sec", 1.0)

        self.input_pose_topic = self.get_parameter("input_pose_topic").value
        self.input_status_topic = self.get_parameter("input_status_topic").value
        self.output_pose_topic = self.get_parameter("output_pose_topic").value
        self.output_odom_topic = self.get_parameter("output_odom_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.output_frame_id = self.get_parameter("output_frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.require_correction_ready = bool(
            self.get_parameter("require_correction_ready").value
        )
        self.status_timeout_sec = max(
            0.0, float(self.get_parameter("status_timeout_sec").value)
        )
        self.use_quality_gate = bool(self.get_parameter("use_quality_gate").value)
        self.fitness_warn = float(self.get_parameter("fitness_warn").value)
        self.log_period_sec = max(0.0, float(self.get_parameter("log_period_sec").value))

        self.state = MatchOriginState(
            min_lock_sec=float(self.get_parameter("min_lock_sec").value),
            min_samples=int(self.get_parameter("min_samples").value),
            max_xy_std=float(self.get_parameter("max_xy_std").value),
            max_yaw_std_rad=math.radians(
                float(self.get_parameter("max_yaw_std_deg").value)
            ),
        )

        self.last_input_wall_sec: float | None = None
        self.last_status_wall_sec: float | None = None
        self.correction_ready = False
        self.latest_fitness = math.nan
        self.latest_quality_fresh = math.nan
        self.latest_reason = REASON_WAIT_INPUT
        self.output_count = 0
        self.reset_count = 0
        self.last_log_wall_sec = 0.0

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 20)
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 20)
        self.status_pub = self.create_publisher(Float32MultiArray, self.status_topic, 10)

        self.create_subscription(PoseStamped, self.input_pose_topic, self.pose_callback, 20)
        self.create_subscription(Float32MultiArray, self.input_status_topic, self.status_callback, 10)
        self.create_service(Trigger, "/robot/reset_match_origin", self.reset_callback)
        self.create_timer(0.2, self.status_timer_callback)

        self.get_logger().info(
            "match origin pose enabled: "
            f"input={self.input_pose_topic}, output={self.output_pose_topic}, "
            f"status={self.status_topic}, min_lock_sec={self.state.min_lock_sec:.2f}, "
            f"min_samples={self.state.min_samples}, max_xy_std={self.state.max_xy_std:.3f}, "
            f"max_yaw_std={math.degrees(self.state.max_yaw_std_rad):.2f}deg"
        )

    def status_callback(self, msg: Float32MultiArray) -> None:
        self.last_status_wall_sec = time.monotonic()
        data = list(msg.data)
        self.correction_ready = len(data) > 1 and data[1] >= 0.5
        self.latest_fitness = data[11] if len(data) > 11 else math.nan
        self.latest_quality_fresh = data[16] if len(data) > 16 else math.nan

    def status_is_fresh(self, now: float) -> bool:
        if self.last_status_wall_sec is None:
            return False
        return now - self.last_status_wall_sec <= self.status_timeout_sec

    def correction_allows_lock(self, now: float) -> bool:
        if not self.require_correction_ready:
            return True
        return self.status_is_fresh(now) and self.correction_ready

    def quality_allows_lock(self) -> bool:
        if not self.use_quality_gate:
            return True
        return math.isfinite(self.latest_fitness) and self.latest_fitness <= self.fitness_warn

    def pose_callback(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        self.last_input_wall_sec = now

        if not self.correction_allows_lock(now):
            if not self.state.locked:
                self.state.samples.clear()
                self.latest_reason = REASON_WAIT_CORRECTION
                self.log_waiting("waiting for continuous pose correction before locking match origin")
            self.publish_status()
            return

        sample = PoseSample(
            wall_sec=now,
            stamp_sec=stamp_to_sec(msg.header.stamp),
            pose=copy_pose(msg.pose),
            yaw=pose_yaw(msg.pose),
        )

        if not self.state.locked:
            if not self.quality_allows_lock():
                self.latest_reason = REASON_QUALITY_GATE
                self.publish_status()
                self.log_waiting(
                    f"waiting for quality gate: fitness={self.latest_fitness:.3f} "
                    f"limit={self.fitness_warn:.3f}"
                )
                return

            decision = self.state.add_sample(sample)
            self.latest_reason = decision.reason
            if decision.locked:
                origin = self.state.origin
                origin_yaw = pose_yaw(origin) if origin is not None else math.nan
                self.get_logger().info(
                    "match origin locked: "
                    f"x={origin.position.x:.4f} y={origin.position.y:.4f} "
                    f"z={origin.position.z:.4f} yaw={origin_yaw:.5f} "
                    f"samples={decision.sample_count} window={decision.window_sec:.2f}s "
                    f"xy_std={decision.xy_std:.4f}m "
                    f"yaw_std={math.degrees(decision.yaw_std):.2f}deg"
                )
            else:
                self.log_lock_progress(decision)
                self.publish_status()
                return

        self.publish_match_pose(msg.header.stamp, sample.pose)
        self.publish_status()

    def publish_match_pose(self, stamp, input_pose: Pose) -> None:
        if self.state.origin is None:
            return
        match_pose = pose_relative_to_planar_origin(self.state.origin, input_pose)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.output_frame_id
        pose_msg.pose = match_pose
        self.pose_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose = match_pose
        self.odom_pub.publish(odom_msg)
        self.output_count += 1

    def reset_callback(self, request, response):
        del request
        self.state.reset()
        self.latest_reason = REASON_COLLECTING
        self.reset_count += 1
        self.get_logger().warn("match origin reset requested; collecting a new stable origin")
        self.publish_status()
        response.success = True
        response.message = "match origin reset; collecting a new stable origin"
        return response

    def status_timer_callback(self) -> None:
        if not self.state.locked and self.last_input_wall_sec is None:
            self.latest_reason = REASON_WAIT_INPUT
        self.publish_status()

    def publish_status(self) -> None:
        now = time.monotonic()
        input_age = now - self.last_input_wall_sec if self.last_input_wall_sec is not None else math.nan
        status_age = now - self.last_status_wall_sec if self.last_status_wall_sec is not None else math.nan
        origin = self.state.origin
        origin_x = origin.position.x if origin is not None else math.nan
        origin_y = origin.position.y if origin is not None else math.nan
        origin_z = origin.position.z if origin is not None else math.nan
        origin_yaw = pose_yaw(origin) if origin is not None else math.nan

        msg = Float32MultiArray()
        msg.data = [
            1.0,
            1.0 if self.state.locked else 0.0,
            float(len(self.state.samples)),
            self.state.window_sec,
            self.state.xy_std,
            math.degrees(self.state.yaw_std) if math.isfinite(self.state.yaw_std) else math.nan,
            1.0 if self.correction_ready else 0.0,
            status_age,
            self.latest_fitness,
            self.latest_quality_fresh,
            float(self.latest_reason),
            origin_x,
            origin_y,
            origin_z,
            origin_yaw,
            float(self.output_count),
            float(self.reset_count),
            input_age,
        ]
        self.status_pub.publish(msg)

    def log_lock_progress(self, decision: LockDecision) -> None:
        reason = {
            REASON_COLLECTING: "collecting",
            REASON_UNSTABLE_XY: "unstable_xy",
            REASON_UNSTABLE_YAW: "unstable_yaw",
        }.get(decision.reason, str(decision.reason))
        self.log_waiting(
            "match origin not locked: "
            f"reason={reason} samples={decision.sample_count}/{self.state.min_samples} "
            f"window={decision.window_sec:.2f}/{self.state.min_lock_sec:.2f}s "
            f"xy_std={decision.xy_std:.4f}/{self.state.max_xy_std:.4f}m "
            f"yaw_std={math.degrees(decision.yaw_std):.2f}/"
            f"{math.degrees(self.state.max_yaw_std_rad):.2f}deg "
            f"fitness={self.latest_fitness:.3f}"
        )

    def log_waiting(self, message: str) -> None:
        now = time.monotonic()
        if self.log_period_sec > 0.0 and now - self.last_log_wall_sec < self.log_period_sec:
            return
        self.last_log_wall_sec = now
        self.get_logger().warn(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MatchOriginPose()
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
