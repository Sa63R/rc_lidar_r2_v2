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

from .pose_math import (
    compose_pose,
    copy_pose,
    inverse_pose,
    interpolate_pose,
    point_from_iter,
    quat_from_rpy,
    rpy_from_quat,
)


def wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pose_yaw(pose: Pose) -> float:
    return rpy_from_quat(pose.orientation)[2]


def pose_delta_xy_yaw(a: Pose, b: Pose) -> tuple[float, float]:
    delta = compose_pose(inverse_pose(a), b)
    xy = math.hypot(delta.position.x, delta.position.y)
    yaw = wrap_angle(pose_yaw(delta))
    return xy, yaw


@dataclass
class OdomSample:
    stamp_sec: float
    wall_sec: float
    pose: Pose


class ContinuousRobotPose(Node):
    """Publish a continuous robot pose from fast odom plus slow map corrections.

    FAST-LIO publishes camera_init -> body continuously. The map localizer publishes
    map -> body at a lower and sometimes stair-step rate. Each accepted map update is
    used only to refresh map -> camera_init; the high-rate output follows FAST-LIO.
    """

    def __init__(self) -> None:
        super().__init__("continuous_robot_pose")

        self.declare_parameter("fast_odom_topic", "/Odometry")
        self.declare_parameter("map_odom_topic", "/map_lidar/odom")
        self.declare_parameter("quality_topic", "/map_lidar/quality")
        self.declare_parameter("output_pose_topic", "/robot/pose_continuous")
        self.declare_parameter("output_odom_topic", "/robot/odom_continuous")
        self.declare_parameter("status_topic", "/robot/pose_continuous/status")
        self.declare_parameter("output_frame_id", "robot_origin")
        self.declare_parameter("child_frame_id", "robot_base")
        self.declare_parameter("output_stamp_mode", "now")
        self.declare_parameter("lidar_to_base_xyz", [-0.240, 0.287302, -0.760])
        self.declare_parameter("lidar_to_base_rpy", [0.0, 0.0, math.pi / 2.0])
        self.declare_parameter("reference_pose_enabled", True)
        self.declare_parameter("reference_pose_xyz", [-0.240, 0.287302, -0.760])
        self.declare_parameter("reference_pose_rpy", [0.0, 0.0, math.pi / 2.0])
        self.declare_parameter("correction_alpha", 0.05)
        self.declare_parameter("odom_buffer_sec", 5.0)
        self.declare_parameter("max_stamp_delta_sec", 0.20)
        self.declare_parameter("stamp_match_mode", "auto")
        self.declare_parameter("quality_gate_enabled", True)
        self.declare_parameter("quality_timeout_sec", 1.0)
        self.declare_parameter("quality_max_fitness", 0.20)
        self.declare_parameter("quality_max_jump_m", 0.08)
        self.declare_parameter("log_period_sec", 1.0)

        self.fast_odom_topic = self.get_parameter("fast_odom_topic").value
        self.map_odom_topic = self.get_parameter("map_odom_topic").value
        self.quality_topic = self.get_parameter("quality_topic").value
        self.output_pose_topic = self.get_parameter("output_pose_topic").value
        self.output_odom_topic = self.get_parameter("output_odom_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.output_frame_id = self.get_parameter("output_frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.output_stamp_mode = str(
            self.get_parameter("output_stamp_mode").value
        ).lower()
        if self.output_stamp_mode not in {"now", "input"}:
            self.get_logger().warn(
                "output_stamp_mode must be 'now' or 'input'; falling back to 'now'"
            )
            self.output_stamp_mode = "now"

        self.correction_alpha = min(
            max(float(self.get_parameter("correction_alpha").value), 0.0), 1.0
        )
        self.odom_buffer_sec = max(
            0.5, float(self.get_parameter("odom_buffer_sec").value)
        )
        self.max_stamp_delta_sec = max(
            0.0, float(self.get_parameter("max_stamp_delta_sec").value)
        )
        self.stamp_match_mode = str(
            self.get_parameter("stamp_match_mode").value
        ).lower()
        if self.stamp_match_mode not in {"auto", "stamp", "latest"}:
            self.get_logger().warn(
                "stamp_match_mode must be auto, stamp, or latest; using auto"
            )
            self.stamp_match_mode = "auto"
        self.quality_gate_enabled = bool(
            self.get_parameter("quality_gate_enabled").value
        )
        self.quality_timeout_sec = float(
            self.get_parameter("quality_timeout_sec").value
        )
        self.quality_max_fitness = float(
            self.get_parameter("quality_max_fitness").value
        )
        self.quality_max_jump_m = float(
            self.get_parameter("quality_max_jump_m").value
        )
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)

        self.body_to_robot_base = Pose()
        self.body_to_robot_base.position = point_from_iter(
            self.get_parameter("lidar_to_base_xyz").value
        )
        lidar_to_base_rpy = [
            float(v) for v in self.get_parameter("lidar_to_base_rpy").value
        ]
        self.body_to_robot_base.orientation = quat_from_rpy(*lidar_to_base_rpy)

        self.reference_pose_enabled = bool(
            self.get_parameter("reference_pose_enabled").value
        )
        self.map_to_robot_origin = Pose()
        self.map_to_robot_origin.position = point_from_iter(
            self.get_parameter("reference_pose_xyz").value
        )
        reference_pose_rpy = [
            float(v) for v in self.get_parameter("reference_pose_rpy").value
        ]
        self.map_to_robot_origin.orientation = quat_from_rpy(*reference_pose_rpy)
        self.robot_origin_to_map = inverse_pose(self.map_to_robot_origin)

        self.odom_buffer: deque[OdomSample] = deque()
        self.map_to_camera_init: Pose | None = None
        self.last_correction_wall_sec: float | None = None
        self.last_fast_wall_sec: float | None = None
        self.last_map_wall_sec: float | None = None
        self.last_quality_wall_sec: float | None = None
        self.last_quality_data: list[float] | None = None
        self.last_status_data: list[float] = []
        self.update_count = 0
        self.rejected_count = 0
        self.output_count = 0
        self.last_log_wall_sec = 0.0

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.status_pub = self.create_publisher(Float32MultiArray, self.status_topic, 10)

        self.fast_sub = self.create_subscription(
            Odometry,
            self.fast_odom_topic,
            self.fast_odom_callback,
            50,
        )
        self.map_sub = self.create_subscription(
            Odometry,
            self.map_odom_topic,
            self.map_odom_callback,
            10,
        )
        self.quality_sub = self.create_subscription(
            Float32MultiArray,
            self.quality_topic,
            self.quality_callback,
            10,
        )

        self.get_logger().info(
            "continuous robot pose enabled: "
            f"fast={self.fast_odom_topic}, map={self.map_odom_topic}, "
            f"output={self.output_pose_topic}, alpha={self.correction_alpha:.3f}, "
            f"stamp_match={self.stamp_match_mode}, max_stamp_delta={self.max_stamp_delta_sec:.3f}s"
        )

    def quality_callback(self, msg: Float32MultiArray) -> None:
        self.last_quality_wall_sec = time.monotonic()
        self.last_quality_data = list(msg.data)

    def quality_values(self) -> tuple[float, float, float, float, float]:
        data = self.last_quality_data or []
        accepted = data[0] if len(data) > 0 else math.nan
        converged = data[1] if len(data) > 1 else math.nan
        fitness = data[2] if len(data) > 2 else math.nan
        jump_xy = data[7] if len(data) > 7 else math.nan
        fresh = 0.0
        if self.last_quality_wall_sec is not None:
            fresh = 1.0 if time.monotonic() - self.last_quality_wall_sec <= self.quality_timeout_sec else 0.0
        return accepted, converged, fitness, jump_xy, fresh

    def quality_allows_update(self) -> tuple[bool, int]:
        if not self.quality_gate_enabled:
            return True, 0

        accepted, converged, fitness, jump_xy, fresh = self.quality_values()
        # Fail open before quality is available; otherwise the correction may never initialize.
        if not math.isfinite(fresh) or fresh < 0.5:
            return True, 1
        if math.isfinite(accepted) and accepted < 0.5:
            return False, 2
        if math.isfinite(converged) and converged < 0.5:
            return False, 3
        if math.isfinite(fitness) and fitness > self.quality_max_fitness:
            return False, 4
        if math.isfinite(jump_xy) and jump_xy > self.quality_max_jump_m:
            return False, 5
        return True, 0

    def fast_odom_callback(self, msg: Odometry) -> None:
        now = time.monotonic()
        fast_pose = copy_pose(msg.pose.pose)
        sample = OdomSample(stamp_to_sec(msg.header.stamp), now, fast_pose)
        self.odom_buffer.append(sample)
        self.last_fast_wall_sec = now
        self.trim_odom_buffer(now)

        if self.map_to_camera_init is None:
            self.publish_status(
                map_update_accepted=0.0,
                reject_reason=6,
                matched_stamp_delta=math.nan,
                candidate_delta_xy=math.nan,
                candidate_delta_yaw=math.nan,
                match_mode_code=0,
            )
            return

        output_pose = self.output_pose_from_fast_pose(fast_pose)
        stamp = msg.header.stamp
        if self.output_stamp_mode == "now":
            stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.output_frame_id
        pose_msg.pose = output_pose
        self.pose_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose = output_pose
        self.odom_pub.publish(odom_msg)
        self.output_count += 1

        self.publish_status(
            map_update_accepted=0.0,
            reject_reason=0,
            matched_stamp_delta=math.nan,
            candidate_delta_xy=math.nan,
            candidate_delta_yaw=math.nan,
            match_mode_code=0,
        )

    def map_odom_callback(self, msg: Odometry) -> None:
        self.last_map_wall_sec = time.monotonic()
        map_body_pose = copy_pose(msg.pose.pose)
        matched_sample, matched_delta_sec, match_mode_code = self.find_matching_odom(
            stamp_to_sec(msg.header.stamp)
        )
        if matched_sample is None:
            self.rejected_count += 1
            self.publish_status(
                map_update_accepted=0.0,
                reject_reason=7,
                matched_stamp_delta=math.nan,
                candidate_delta_xy=math.nan,
                candidate_delta_yaw=math.nan,
                match_mode_code=0,
            )
            return

        quality_ok, reject_reason = self.quality_allows_update()
        candidate = compose_pose(map_body_pose, inverse_pose(matched_sample.pose))
        candidate_delta_xy = 0.0
        candidate_delta_yaw = 0.0
        if self.map_to_camera_init is not None:
            candidate_delta_xy, candidate_delta_yaw = pose_delta_xy_yaw(
                self.map_to_camera_init, candidate
            )

        if not quality_ok:
            self.rejected_count += 1
            self.publish_status(
                map_update_accepted=0.0,
                reject_reason=reject_reason,
                matched_stamp_delta=matched_delta_sec,
                candidate_delta_xy=candidate_delta_xy,
                candidate_delta_yaw=candidate_delta_yaw,
                match_mode_code=match_mode_code,
            )
            return

        if self.map_to_camera_init is None:
            self.map_to_camera_init = candidate
        else:
            self.map_to_camera_init = interpolate_pose(
                self.map_to_camera_init,
                candidate,
                self.correction_alpha,
            )
        self.last_correction_wall_sec = time.monotonic()
        self.update_count += 1

        self.publish_status(
            map_update_accepted=1.0,
            reject_reason=0,
            matched_stamp_delta=matched_delta_sec,
            candidate_delta_xy=candidate_delta_xy,
            candidate_delta_yaw=candidate_delta_yaw,
            match_mode_code=match_mode_code,
        )
        self.log_map_update(candidate_delta_xy, candidate_delta_yaw, matched_delta_sec)

    def trim_odom_buffer(self, now: float) -> None:
        while self.odom_buffer and now - self.odom_buffer[0].wall_sec > self.odom_buffer_sec:
            self.odom_buffer.popleft()

    def find_matching_odom(self, map_stamp_sec: float) -> tuple[OdomSample | None, float, int]:
        if not self.odom_buffer:
            return None, math.nan, 0

        latest = self.odom_buffer[-1]
        if self.stamp_match_mode == "latest" or map_stamp_sec <= 0.0:
            return latest, math.nan, 2

        best = min(
            self.odom_buffer,
            key=lambda sample: abs(sample.stamp_sec - map_stamp_sec)
            if sample.stamp_sec > 0.0
            else float("inf"),
        )
        best_delta = abs(best.stamp_sec - map_stamp_sec) if best.stamp_sec > 0.0 else math.nan

        if math.isfinite(best_delta) and best_delta <= self.max_stamp_delta_sec:
            return best, best_delta, 1
        if self.stamp_match_mode == "auto":
            return latest, best_delta, 2
        return None, best_delta, 3

    def output_pose_from_fast_pose(self, fast_body_pose: Pose) -> Pose:
        map_body = compose_pose(self.map_to_camera_init, fast_body_pose)
        map_robot_base = compose_pose(map_body, self.body_to_robot_base)
        if self.reference_pose_enabled:
            return compose_pose(self.robot_origin_to_map, map_robot_base)
        return map_robot_base

    def publish_status(
        self,
        *,
        map_update_accepted: float,
        reject_reason: int,
        matched_stamp_delta: float,
        candidate_delta_xy: float,
        candidate_delta_yaw: float,
        match_mode_code: int,
    ) -> None:
        now = time.monotonic()
        correction_ready = 1.0 if self.map_to_camera_init is not None else 0.0
        correction_age = (
            now - self.last_correction_wall_sec
            if self.last_correction_wall_sec is not None
            else math.nan
        )
        odom_age = now - self.last_fast_wall_sec if self.last_fast_wall_sec else math.nan
        map_age = now - self.last_map_wall_sec if self.last_map_wall_sec else math.nan
        accepted, converged, fitness, jump_xy, quality_fresh = self.quality_values()
        data = [
            1.0,
            correction_ready,
            float(map_update_accepted),
            self.correction_alpha,
            correction_age,
            odom_age,
            matched_stamp_delta,
            candidate_delta_xy,
            math.degrees(candidate_delta_yaw) if math.isfinite(candidate_delta_yaw) else math.nan,
            accepted,
            converged,
            fitness,
            jump_xy,
            float(self.update_count),
            float(self.rejected_count),
            float(match_mode_code),
            quality_fresh,
            map_age,
            float(self.output_count),
            float(reject_reason),
        ]
        self.last_status_data = data
        msg = Float32MultiArray()
        msg.data = data
        self.status_pub.publish(msg)

    def log_map_update(
        self,
        candidate_delta_xy: float,
        candidate_delta_yaw: float,
        matched_stamp_delta: float,
    ) -> None:
        now = time.monotonic()
        if now - self.last_log_wall_sec < self.log_period_sec:
            return
        self.last_log_wall_sec = now
        self.get_logger().info(
            "map correction accepted: "
            f"updates={self.update_count} alpha={self.correction_alpha:.3f} "
            f"candidate_delta_xy={candidate_delta_xy:.4f}m "
            f"candidate_delta_yaw={math.degrees(candidate_delta_yaw):.2f}deg "
            f"matched_dt={matched_stamp_delta:.4f}s"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ContinuousRobotPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
