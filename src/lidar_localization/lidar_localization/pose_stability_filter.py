import math
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray

from .pose_math import quat_from_rpy, rpy_from_quat


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class PoseStabilityFilter(Node):
    """Low-latency pose smoother for observing relocalization pose stability."""

    def __init__(self) -> None:
        super().__init__("pose_stability_filter")

        self.input_pose_topic = self.declare_parameter(
            "input_pose_topic", "/lidar/pose"
        ).value
        self.output_pose_topic = self.declare_parameter(
            "output_pose_topic", "/lidar/pose_filtered"
        ).value
        self.output_pose_alias_topic = self.declare_parameter(
            "output_pose_alias_topic", "/robot/pose_filtered"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/lidar/pose_filter/status"
        ).value
        self.static_alpha = float(self.declare_parameter("static_alpha", 0.12).value)
        self.moving_alpha = float(self.declare_parameter("moving_alpha", 0.92).value)
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
        self.recovery_reject_count = int(
            self.declare_parameter("recovery_reject_count", 10).value
        )
        self.recovery_time_sec = float(
            self.declare_parameter("recovery_time_sec", 1.0).value
        )
        self.log_period_sec = float(
            self.declare_parameter("log_period_sec", 1.0).value
        )

        self.filtered_pose: Optional[PoseStamped] = None
        self.last_raw_pose: Optional[PoseStamped] = None
        self.last_raw_stamp: Optional[Time] = None
        self.last_receive_time: Optional[Time] = None
        self.first_reject_time: Optional[Time] = None
        self.consecutive_rejects = 0
        self.reject_count = 0
        self.accept_count = 0
        self.last_log_time = self.get_clock().now()

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 20)
        self.pose_alias_pub = (
            self.create_publisher(PoseStamped, self.output_pose_alias_topic, 20)
            if self.output_pose_alias_topic
            else None
        )
        self.status_pub = self.create_publisher(Float32MultiArray, self.status_topic, 20)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.input_pose_topic, self.pose_callback, 20
        )

        self.get_logger().info(
            f"pose stability filter ready: {self.input_pose_topic} -> {self.output_pose_topic}"
            f"{f' and {self.output_pose_alias_topic}' if self.output_pose_alias_topic else ''}"
        )
        self.get_logger().info(
            "filter params: "
            f"static_alpha={self.static_alpha:.3f} "
            f"moving_alpha={self.moving_alpha:.3f} "
            f"static_speed={self.static_speed_threshold:.3f} "
            f"full_speed={self.full_speed_threshold:.3f} "
            f"static_yaw_rate_deg={math.degrees(self.static_yaw_rate_rad):.1f} "
            f"full_yaw_rate_deg={math.degrees(self.full_yaw_rate_rad):.1f} "
            f"max_xy_jump={self.max_xy_jump:.3f} "
            f"max_xy_speed={self.max_xy_speed:.3f} "
            f"max_yaw_jump_deg={math.degrees(self.max_yaw_jump_rad):.1f} "
            f"recovery_reject_count={self.recovery_reject_count} "
            f"recovery_time={self.recovery_time_sec:.2f}s"
        )

    def pose_stamp(self, msg: PoseStamped) -> Time:
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            return self.get_clock().now()
        return stamp

    def pose_callback(self, msg: PoseStamped) -> None:
        receive_time = self.get_clock().now()
        stamp = self.pose_stamp(msg)
        roll, pitch, yaw = rpy_from_quat(msg.pose.orientation)

        if self.filtered_pose is None or self.last_raw_pose is None:
            self.filtered_pose = self.copy_pose(msg)
            self.last_raw_pose = self.copy_pose(msg)
            self.last_raw_stamp = stamp
            self.last_receive_time = receive_time
            self.accept_count += 1
            self.publish_filtered_pose(self.filtered_pose)
            self.publish_status(
                True,
                msg,
                self.filtered_pose,
                0.0,
                0.0,
                1.0,
                yaw,
                yaw,
                0.0,
                0.0,
            )
            return

        prev_raw = self.last_raw_pose
        prev_stamp = self.last_raw_stamp or stamp
        dt_from_stamp = (stamp - prev_stamp).nanoseconds / 1e9
        dt_from_wall = 0.0
        if self.last_receive_time is not None:
            dt_from_wall = (receive_time - self.last_receive_time).nanoseconds / 1e9
        dt = dt_from_stamp if dt_from_stamp > 1e-3 else dt_from_wall
        dt = max(dt, 1e-3)

        dx = msg.pose.position.x - prev_raw.pose.position.x
        dy = msg.pose.position.y - prev_raw.pose.position.y
        xy_step = math.hypot(dx, dy)
        speed_xy = xy_step / dt

        _, _, prev_yaw = rpy_from_quat(prev_raw.pose.orientation)
        yaw_step = abs(wrap_angle(yaw - prev_yaw))
        yaw_rate = yaw_step / dt
        motion_factor = self.motion_factor(speed_xy, yaw_rate)
        alpha = self.alpha_for_motion(motion_factor)

        rejected = (
            (xy_step > self.max_xy_jump and speed_xy > self.max_xy_speed)
            or yaw_step > self.max_yaw_jump_rad
        )

        if rejected:
            self.reject_count += 1
            self.consecutive_rejects += 1
            if self.first_reject_time is None:
                self.first_reject_time = receive_time
            reject_duration = (
                receive_time - self.first_reject_time
            ).nanoseconds / 1e9

            if self.should_recover_from_rejects(reject_duration):
                self.accept_count += 1
                self.consecutive_rejects = 0
                self.first_reject_time = None
                self.filtered_pose = self.copy_pose(msg)
                self.last_raw_pose = self.copy_pose(msg)
                self.last_raw_stamp = stamp
                self.last_receive_time = receive_time
                self.publish_filtered_pose(self.filtered_pose)
                self.publish_status(
                    True,
                    msg,
                    self.filtered_pose,
                    xy_step,
                    speed_xy,
                    1.0,
                    yaw,
                    yaw,
                    motion_factor,
                    yaw_rate,
                    recovered=True,
                )
                self.log_recover(xy_step, speed_xy, yaw_step, reject_duration)
                return

            self.last_receive_time = receive_time
            output = self.copy_pose(self.filtered_pose)
            output.header = msg.header
            self.publish_filtered_pose(output)
            _, _, filtered_yaw = rpy_from_quat(output.pose.orientation)
            self.publish_status(
                False,
                msg,
                output,
                xy_step,
                speed_xy,
                0.0,
                yaw,
                filtered_yaw,
                motion_factor,
                yaw_rate,
            )
            self.log_reject(xy_step, speed_xy, yaw_step, reject_duration)
            return

        self.accept_count += 1
        self.consecutive_rejects = 0
        self.first_reject_time = None

        output = self.copy_pose(msg)
        prev_filtered = self.filtered_pose
        output.pose.position.x = self.ema(
            prev_filtered.pose.position.x, msg.pose.position.x, alpha
        )
        output.pose.position.y = self.ema(
            prev_filtered.pose.position.y, msg.pose.position.y, alpha
        )
        output.pose.position.z = self.ema(
            prev_filtered.pose.position.z, msg.pose.position.z, self.static_alpha
        )

        prev_roll, prev_pitch, prev_filtered_yaw = rpy_from_quat(
            prev_filtered.pose.orientation
        )
        filtered_yaw = wrap_angle(prev_filtered_yaw + alpha * wrap_angle(yaw - prev_filtered_yaw))
        filtered_roll = self.ema(prev_roll, roll, self.static_alpha)
        filtered_pitch = self.ema(prev_pitch, pitch, self.static_alpha)
        output.pose.orientation = quat_from_rpy(filtered_roll, filtered_pitch, filtered_yaw)

        self.filtered_pose = output
        self.last_raw_pose = self.copy_pose(msg)
        self.last_raw_stamp = stamp
        self.last_receive_time = receive_time
        self.publish_filtered_pose(output)
        self.publish_status(
            True,
            msg,
            output,
            xy_step,
            speed_xy,
            alpha,
            yaw,
            filtered_yaw,
            motion_factor,
            yaw_rate,
        )
        self.log_accept(xy_step, speed_xy, alpha, motion_factor, yaw_rate)

    def motion_factor(self, speed_xy: float, yaw_rate: float) -> float:
        speed_factor = self.ramp(
            speed_xy, self.static_speed_threshold, self.full_speed_threshold
        )
        yaw_factor = self.ramp(
            yaw_rate, self.static_yaw_rate_rad, self.full_yaw_rate_rad
        )
        return max(speed_factor, yaw_factor)

    def alpha_for_motion(self, motion_factor: float) -> float:
        static_alpha = min(max(self.static_alpha, 0.0), 1.0)
        moving_alpha = min(max(self.moving_alpha, 0.0), 1.0)
        factor = min(max(motion_factor, 0.0), 1.0)
        return static_alpha + factor * (moving_alpha - static_alpha)

    def should_recover_from_rejects(self, reject_duration: float) -> bool:
        count_reached = (
            self.recovery_reject_count > 0
            and self.consecutive_rejects >= self.recovery_reject_count
        )
        time_reached = (
            self.recovery_time_sec > 0.0
            and reject_duration >= self.recovery_time_sec
        )
        return count_reached or time_reached

    @staticmethod
    def ema(previous: float, current: float, alpha: float) -> float:
        return previous + alpha * (current - previous)

    @staticmethod
    def ramp(value: float, low: float, high: float) -> float:
        if high <= low:
            return 1.0 if value >= high else 0.0
        return min(max((value - low) / (high - low), 0.0), 1.0)

    def publish_filtered_pose(self, msg: PoseStamped) -> None:
        self.pose_pub.publish(msg)
        if self.pose_alias_pub:
            self.pose_alias_pub.publish(msg)

    @staticmethod
    def copy_pose(msg: PoseStamped) -> PoseStamped:
        copied = PoseStamped()
        copied.header = msg.header
        copied.pose.position.x = msg.pose.position.x
        copied.pose.position.y = msg.pose.position.y
        copied.pose.position.z = msg.pose.position.z
        copied.pose.orientation.x = msg.pose.orientation.x
        copied.pose.orientation.y = msg.pose.orientation.y
        copied.pose.orientation.z = msg.pose.orientation.z
        copied.pose.orientation.w = msg.pose.orientation.w
        return copied

    def publish_status(
        self,
        accepted: bool,
        raw: PoseStamped,
        filtered: PoseStamped,
        xy_step: float,
        speed_xy: float,
        alpha: float,
        raw_yaw: float,
        filtered_yaw: float,
        motion_factor: float,
        yaw_rate: float,
        recovered: bool = False,
    ) -> None:
        status = Float32MultiArray()
        status.data = [
            1.0 if accepted else 0.0,
            float(raw.pose.position.x),
            float(raw.pose.position.y),
            float(filtered.pose.position.x),
            float(filtered.pose.position.y),
            float(xy_step),
            float(speed_xy),
            float(alpha),
            float(raw_yaw),
            float(filtered_yaw),
            float(self.reject_count),
            float(self.accept_count),
            float(self.consecutive_rejects),
            1.0 if recovered else 0.0,
            float(motion_factor),
            float(math.degrees(yaw_rate)),
        ]
        self.status_pub.publish(status)

    def log_accept(
        self,
        xy_step: float,
        speed_xy: float,
        alpha: float,
        motion_factor: float,
        yaw_rate: float,
    ) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 < self.log_period_sec:
            return
        self.last_log_time = now
        self.get_logger().info(
            f"pose filter accepted: step={xy_step:.4f}m "
            f"speed={speed_xy:.3f}m/s alpha={alpha:.2f} "
            f"motion={motion_factor:.2f} yaw_rate={math.degrees(yaw_rate):.1f}deg/s "
            f"rejects={self.reject_count}"
        )

    def log_reject(
        self, xy_step: float, speed_xy: float, yaw_step: float, reject_duration: float
    ) -> None:
        self.get_logger().warn(
            f"pose filter rejected jump: xy_step={xy_step:.3f}m "
            f"speed={speed_xy:.3f}m/s yaw_step={math.degrees(yaw_step):.1f}deg "
            f"rejects={self.reject_count} consecutive={self.consecutive_rejects} "
            f"duration={reject_duration:.2f}s"
        )

    def log_recover(
        self, xy_step: float, speed_xy: float, yaw_step: float, reject_duration: float
    ) -> None:
        self.get_logger().warn(
            "pose filter recovered by rebasing after persistent jump: "
            f"xy_step={xy_step:.3f}m speed={speed_xy:.3f}m/s "
            f"yaw_step={math.degrees(yaw_step):.1f}deg "
            f"duration={reject_duration:.2f}s"
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
