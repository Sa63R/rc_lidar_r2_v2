#!/usr/bin/env python3
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .pose_math import (
    point_from_iter,
    quat_from_rpy,
    quat_inverse,
    quat_multiply,
    rotate_point,
    rpy_from_quat,
)


def wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def copy_pose(pose: Pose) -> Pose:
    copied = Pose()
    copied.position.x = pose.position.x
    copied.position.y = pose.position.y
    copied.position.z = pose.position.z
    copied.orientation.x = pose.orientation.x
    copied.orientation.y = pose.orientation.y
    copied.orientation.z = pose.orientation.z
    copied.orientation.w = pose.orientation.w
    return copied


def pose_yaw(pose: Pose) -> float:
    return rpy_from_quat(pose.orientation)[2]


def pose_with_yaw(pose: Pose, yaw: float) -> Pose:
    updated = copy_pose(pose)
    roll, pitch, _ = rpy_from_quat(pose.orientation)
    updated.orientation = quat_from_rpy(roll, pitch, wrap_angle(yaw))
    return updated


def pose_relative_to_reference(pose: Pose, reference: Pose) -> Pose:
    delta = PointStamped().point
    delta.x = pose.position.x - reference.position.x
    delta.y = pose.position.y - reference.position.y
    delta.z = pose.position.z - reference.position.z

    reference_inv = quat_inverse(reference.orientation)
    relative_position = rotate_point(reference_inv, delta)

    relative_pose = Pose()
    relative_pose.position = relative_position
    relative_pose.orientation = quat_multiply(reference_inv, pose.orientation)
    return relative_pose


class PoseFromOdom(Node):
    """Expose a stable LiDAR pose interface from a LIO/SLAM odometry topic."""

    def __init__(self) -> None:
        super().__init__("lidar_pose_from_odom")

        self.declare_parameter("input_odom_topic", "/Odometry")
        self.declare_parameter("output_pose_topic", "/lidar/pose")
        self.declare_parameter("output_pose_alias_topic", "/robot/pose")
        self.declare_parameter("output_point_topic", "/lidar/point")
        self.declare_parameter("output_odom_topic", "/lidar/odom")
        self.declare_parameter("lidar_frame", "body")
        self.declare_parameter("base_frame", "robot_base")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("publish_static_tf", False)
        self.declare_parameter("output_base_frame", True)
        self.declare_parameter("zero_on_first_pose", True)
        self.declare_parameter("zero_delay_sec", 5.0)
        self.declare_parameter("zero_frame_id", "robot_origin")
        self.declare_parameter("lidar_to_base_xyz", [0.237488, -0.272804, -0.483])
        self.declare_parameter("lidar_to_base_rpy", [0.0, 0.0, -math.pi / 2.0])
        self.declare_parameter("reference_pose_enabled", True)
        self.declare_parameter("reference_frame_id", "robot_origin")
        self.declare_parameter("reference_pose_xyz", [-0.260, 0.262, -0.483])
        self.declare_parameter("reference_pose_rpy", [0.0, 0.0, -math.pi / 2.0])
        self.declare_parameter("output_stamp_mode", "now")
        self.declare_parameter("log_period_sec", 1.0)
        self.declare_parameter("yaw_fusion_enabled", False)
        self.declare_parameter("fast_odom_topic", "/Odometry")
        self.declare_parameter("yaw_fusion_publish_on_fast_odom", True)
        self.declare_parameter("yaw_fusion_max_fast_age_sec", 0.50)
        self.declare_parameter("yaw_fusion_map_correction_alpha", 0.05)
        self.declare_parameter("yaw_fusion_status_topic", "/robot/yaw_fusion/status")

        self.input_odom_topic = self.get_parameter("input_odom_topic").value
        self.output_pose_topic = self.get_parameter("output_pose_topic").value
        self.output_pose_alias_topic = self.get_parameter("output_pose_alias_topic").value
        self.output_point_topic = self.get_parameter("output_point_topic").value
        self.output_odom_topic = self.get_parameter("output_odom_topic").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_static_tf = bool(self.get_parameter("publish_static_tf").value)
        self.output_base_frame = bool(self.get_parameter("output_base_frame").value)
        self.zero_on_first_pose = bool(self.get_parameter("zero_on_first_pose").value)
        self.zero_delay_sec = float(self.get_parameter("zero_delay_sec").value)
        self.zero_frame_id = self.get_parameter("zero_frame_id").value
        self.reference_pose_enabled = bool(
            self.get_parameter("reference_pose_enabled").value
        )
        self.reference_frame_id = self.get_parameter("reference_frame_id").value
        self.output_stamp_mode = str(
            self.get_parameter("output_stamp_mode").value
        ).lower()
        if self.output_stamp_mode not in {"now", "input"}:
            self.get_logger().warn(
                "output_stamp_mode must be 'now' or 'input'; falling back to 'now'"
            )
            self.output_stamp_mode = "now"
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)
        self.initial_pose: Pose | None = None
        self.pending_origin_pose: Pose | None = None
        self.first_odom_time = None
        self.latest_pose: Pose | None = None
        self.yaw_fusion_enabled = bool(self.get_parameter("yaw_fusion_enabled").value)
        self.fast_odom_topic = self.get_parameter("fast_odom_topic").value
        self.yaw_fusion_publish_on_fast_odom = bool(
            self.get_parameter("yaw_fusion_publish_on_fast_odom").value
        )
        self.yaw_fusion_max_fast_age_sec = float(
            self.get_parameter("yaw_fusion_max_fast_age_sec").value
        )
        self.yaw_fusion_map_correction_alpha = min(
            max(float(self.get_parameter("yaw_fusion_map_correction_alpha").value), 0.0),
            1.0,
        )
        self.yaw_fusion_status_topic = self.get_parameter("yaw_fusion_status_topic").value

        self.latest_fast_robot_yaw: float | None = None
        self.latest_fast_wall_sec: float | None = None
        self.latest_map_wall_sec: float | None = None
        self.yaw_fusion_map_ref_yaw: float | None = None
        self.yaw_fusion_fast_ref_yaw: float | None = None
        self.latest_output_pose: Pose | None = None
        self.latest_frame_id = self.reference_frame_id
        self.latest_child_frame = self.base_frame if self.output_base_frame else self.lidar_frame
        self.latest_covariance = None
        self.latest_twist = None

        self.lidar_to_base_translation = point_from_iter(
            self.get_parameter("lidar_to_base_xyz").value
        )
        lidar_to_base_rpy = [float(v) for v in self.get_parameter("lidar_to_base_rpy").value]
        self.lidar_to_base_rotation = quat_from_rpy(*lidar_to_base_rpy)
        self.reference_pose = Pose()
        self.reference_pose.position = point_from_iter(
            self.get_parameter("reference_pose_xyz").value
        )
        reference_pose_rpy = [
            float(v) for v in self.get_parameter("reference_pose_rpy").value
        ]
        self.reference_pose.orientation = quat_from_rpy(*reference_pose_rpy)

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.pose_alias_pub = (
            self.create_publisher(PoseStamped, self.output_pose_alias_topic, 10)
            if self.output_pose_alias_topic
            else None
        )
        self.point_pub = self.create_publisher(PointStamped, self.output_point_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.yaw_fusion_status_pub = self.create_publisher(
            Float32MultiArray,
            self.yaw_fusion_status_topic,
            10,
        )
        self.tf_broadcaster: Optional[TransformBroadcaster] = (
            TransformBroadcaster(self) if self.publish_tf else None
        )
        self.static_tf_broadcaster: Optional[StaticTransformBroadcaster] = (
            StaticTransformBroadcaster(self) if self.publish_static_tf else None
        )
        if self.static_tf_broadcaster:
            self.broadcast_static_base_to_lidar_tf()

        self.last_log_time = self.get_clock().now()
        self.subscription = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self.odom_callback,
            20,
        )
        self.fast_subscription = None
        if self.yaw_fusion_enabled:
            self.fast_subscription = self.create_subscription(
                Odometry,
                self.fast_odom_topic,
                self.fast_odom_callback,
                50,
            )
        self.reset_origin_srv = self.create_service(
            Trigger,
            "/lidar/reset_origin",
            self.reset_origin_callback,
        )

        target = self.base_frame if self.output_base_frame else self.lidar_frame
        self.get_logger().info(
            f"waiting for {self.input_odom_topic}; publishing {target} pose on "
            f"{self.output_pose_topic}"
            f"{f' and {self.output_pose_alias_topic}' if self.output_pose_alias_topic else ''}"
            f"; point on {self.output_point_topic}"
        )
        if self.yaw_fusion_enabled:
            self.get_logger().info(
                "low-latency yaw fusion enabled: "
                f"map topic={self.input_odom_topic}, fast topic={self.fast_odom_topic}, "
                f"publish_on_fast_odom={self.yaw_fusion_publish_on_fast_odom}, "
                f"map_correction_alpha={self.yaw_fusion_map_correction_alpha:.3f}"
            )

    def odom_callback(self, msg: Odometry) -> None:
        pose = copy_pose(msg.pose.pose)
        child_frame = self.lidar_frame

        if self.output_base_frame:
            pose = self.compose_lidar_to_base(pose)
            child_frame = self.base_frame
        self.latest_pose = pose

        frame_id = msg.header.frame_id
        if self.zero_on_first_pose:
            self.update_initial_pose(pose)
            pose = self.pose_relative_to_initial(pose)
            frame_id = self.zero_frame_id
        elif self.reference_pose_enabled:
            pose = pose_relative_to_reference(pose, self.reference_pose)
            frame_id = self.reference_frame_id

        self.latest_map_wall_sec = time.monotonic()
        self.latest_covariance = msg.pose.covariance
        self.latest_twist = msg.twist
        if self.yaw_fusion_enabled:
            pose = self.update_yaw_fusion_reference_from_map(pose)

        stamp = msg.header.stamp
        if self.output_stamp_mode == "now":
            stamp = self.get_clock().now().to_msg()
        self.publish_output_pose(pose, frame_id, child_frame, stamp, source_code=1.0)

    def fast_odom_callback(self, msg: Odometry) -> None:
        fast_pose = copy_pose(msg.pose.pose)
        if self.output_base_frame:
            fast_pose = self.compose_lidar_to_base(fast_pose)
        self.latest_fast_robot_yaw = pose_yaw(fast_pose)
        self.latest_fast_wall_sec = time.monotonic()
        if (
            self.yaw_fusion_enabled
            and self.yaw_fusion_map_ref_yaw is not None
            and self.yaw_fusion_fast_ref_yaw is None
        ):
            self.yaw_fusion_fast_ref_yaw = self.latest_fast_robot_yaw

        if (
            not self.yaw_fusion_enabled
            or not self.yaw_fusion_publish_on_fast_odom
            or self.latest_output_pose is None
            or not self.yaw_fusion_ready()
        ):
            self.publish_yaw_fusion_status(source_code=2.0, fused_yaw=math.nan)
            return

        fused_pose = self.pose_with_fused_yaw(self.latest_output_pose)
        stamp = msg.header.stamp
        if self.output_stamp_mode == "now":
            stamp = self.get_clock().now().to_msg()
        self.publish_output_pose(
            fused_pose,
            self.latest_frame_id,
            self.latest_child_frame,
            stamp,
            source_code=2.0,
        )

    def update_initial_pose(self, pose: Pose) -> None:
        if self.initial_pose is not None:
            return

        now = self.get_clock().now()
        if self.first_odom_time is None:
            self.first_odom_time = now
            self.pending_origin_pose = pose
            self.get_logger().info(
                f"waiting {self.zero_delay_sec:.1f}s before locking output origin"
            )

        elapsed_sec = (now - self.first_odom_time).nanoseconds / 1e9
        if elapsed_sec >= self.zero_delay_sec:
            self.initial_pose = pose
            self.get_logger().info("output origin locked from current pose")

    def reset_origin_callback(self, request, response):
        del request
        if self.latest_pose is None:
            response.success = False
            response.message = "No odometry pose received yet."
            return response

        self.initial_pose = self.latest_pose
        self.first_odom_time = self.get_clock().now()
        response.success = True
        response.message = "LiDAR output origin reset to current pose."
        self.get_logger().info(response.message)
        return response

    def compose_lidar_to_base(self, lidar_pose: Pose) -> Pose:
        rotated_offset = rotate_point(
            lidar_pose.orientation, self.lidar_to_base_translation
        )
        base_pose = Pose()
        base_pose.position.x = lidar_pose.position.x + rotated_offset.x
        base_pose.position.y = lidar_pose.position.y + rotated_offset.y
        base_pose.position.z = lidar_pose.position.z + rotated_offset.z
        base_pose.orientation = quat_multiply(
            lidar_pose.orientation, self.lidar_to_base_rotation
        )
        return base_pose

    def fast_odom_is_fresh(self) -> bool:
        if self.latest_fast_wall_sec is None:
            return False
        age = time.monotonic() - self.latest_fast_wall_sec
        return age <= self.yaw_fusion_max_fast_age_sec

    def yaw_fusion_ready(self) -> bool:
        return (
            self.yaw_fusion_map_ref_yaw is not None
            and self.yaw_fusion_fast_ref_yaw is not None
            and self.latest_fast_robot_yaw is not None
            and self.fast_odom_is_fresh()
        )

    def fused_yaw(self) -> float | None:
        if not self.yaw_fusion_ready():
            return None
        delta = wrap_angle(self.latest_fast_robot_yaw - self.yaw_fusion_fast_ref_yaw)
        return wrap_angle(self.yaw_fusion_map_ref_yaw + delta)

    def pose_with_fused_yaw(self, pose: Pose) -> Pose:
        yaw = self.fused_yaw()
        if yaw is None:
            return pose
        return pose_with_yaw(pose, yaw)

    def update_yaw_fusion_reference_from_map(self, map_pose: Pose) -> Pose:
        if self.latest_fast_robot_yaw is None or not self.fast_odom_is_fresh():
            self.yaw_fusion_map_ref_yaw = pose_yaw(map_pose)
            self.yaw_fusion_fast_ref_yaw = None
            self.publish_yaw_fusion_status(source_code=1.0, fused_yaw=self.yaw_fusion_map_ref_yaw)
            return map_pose

        map_yaw = pose_yaw(map_pose)
        if not self.yaw_fusion_ready():
            self.yaw_fusion_map_ref_yaw = map_yaw
            self.yaw_fusion_fast_ref_yaw = self.latest_fast_robot_yaw
            fused = map_yaw
        else:
            predicted = self.fused_yaw()
            if predicted is None:
                predicted = map_yaw
            correction = wrap_angle(map_yaw - predicted)
            fused = wrap_angle(
                predicted + self.yaw_fusion_map_correction_alpha * correction
            )
            self.yaw_fusion_map_ref_yaw = fused
            self.yaw_fusion_fast_ref_yaw = self.latest_fast_robot_yaw

        fused_pose = pose_with_yaw(map_pose, fused)
        self.publish_yaw_fusion_status(source_code=1.0, fused_yaw=fused)
        return fused_pose

    def publish_output_pose(
        self,
        pose: Pose,
        frame_id: str,
        child_frame: str,
        stamp,
        source_code: float,
    ) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = stamp
        pose_msg.pose = pose
        self.pose_pub.publish(pose_msg)
        if self.pose_alias_pub:
            self.pose_alias_pub.publish(pose_msg)

        point_msg = PointStamped()
        point_msg.header = pose_msg.header
        point_msg.point = pose.position
        self.point_pub.publish(point_msg)

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = child_frame
        odom_msg.pose.pose = pose
        if self.latest_covariance is not None:
            odom_msg.pose.covariance = self.latest_covariance
        if self.latest_twist is not None:
            odom_msg.twist = self.latest_twist
        self.odom_pub.publish(odom_msg)

        if self.tf_broadcaster:
            self.broadcast_tf(frame_id, child_frame, pose_msg)

        self.latest_output_pose = copy_pose(pose)
        self.latest_frame_id = frame_id
        self.latest_child_frame = child_frame
        self.publish_yaw_fusion_status(source_code=source_code, fused_yaw=pose_yaw(pose))
        self.log_pose(pose_msg, child_frame)

    def publish_yaw_fusion_status(self, source_code: float, fused_yaw: float) -> None:
        now = time.monotonic()
        fast_age = (
            now - self.latest_fast_wall_sec
            if self.latest_fast_wall_sec is not None
            else math.nan
        )
        map_age = (
            now - self.latest_map_wall_sec
            if self.latest_map_wall_sec is not None
            else math.nan
        )
        fast_yaw = (
            self.latest_fast_robot_yaw
            if self.latest_fast_robot_yaw is not None
            else math.nan
        )
        map_ref = (
            self.yaw_fusion_map_ref_yaw
            if self.yaw_fusion_map_ref_yaw is not None
            else math.nan
        )
        fast_ref = (
            self.yaw_fusion_fast_ref_yaw
            if self.yaw_fusion_fast_ref_yaw is not None
            else math.nan
        )
        delta_yaw = (
            wrap_angle(fast_yaw - fast_ref)
            if math.isfinite(fast_yaw) and math.isfinite(fast_ref)
            else math.nan
        )
        self.yaw_fusion_status_pub.publish(
            Float32MultiArray(
                data=[
                    1.0 if self.yaw_fusion_enabled else 0.0,
                    1.0 if self.yaw_fusion_ready() else 0.0,
                    float(source_code),
                    float(fast_age),
                    float(map_age),
                    float(map_ref),
                    float(fast_yaw),
                    float(fused_yaw),
                    float(delta_yaw),
                    float(self.yaw_fusion_map_correction_alpha),
                ]
            )
        )

    def broadcast_static_base_to_lidar_tf(self) -> None:
        base_to_lidar_rotation = quat_inverse(self.lidar_to_base_rotation)

        negative_offset = PointStamped().point
        negative_offset.x = -self.lidar_to_base_translation.x
        negative_offset.y = -self.lidar_to_base_translation.y
        negative_offset.z = -self.lidar_to_base_translation.z
        base_to_lidar_translation = rotate_point(base_to_lidar_rotation, negative_offset)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.lidar_frame
        transform.transform.translation.x = base_to_lidar_translation.x
        transform.transform.translation.y = base_to_lidar_translation.y
        transform.transform.translation.z = base_to_lidar_translation.z
        transform.transform.rotation = base_to_lidar_rotation
        self.static_tf_broadcaster.sendTransform(transform)

    def pose_relative_to_initial(self, pose: Pose) -> Pose:
        if self.initial_pose is None:
            zero_pose = Pose()
            zero_pose.orientation.w = 1.0
            return zero_pose

        return pose_relative_to_reference(pose, self.initial_pose)

    def broadcast_tf(self, parent_frame: str, child_frame: str, pose_msg: PoseStamped) -> None:
        transform = TransformStamped()
        transform.header = pose_msg.header
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = pose_msg.pose.position.x
        transform.transform.translation.y = pose_msg.pose.position.y
        transform.transform.translation.z = pose_msg.pose.position.z
        transform.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def log_pose(self, pose_msg: PoseStamped, child_frame: str) -> None:
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < self.log_period_sec * 1e9:
            return
        self.last_log_time = now

        roll, pitch, yaw = rpy_from_quat(pose_msg.pose.orientation)
        p = pose_msg.pose.position
        self.get_logger().info(
            f"{child_frame} in {pose_msg.header.frame_id}: "
            f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} "
            f"roll={math.degrees(roll):.2f}deg "
            f"pitch={math.degrees(pitch):.2f}deg "
            f"yaw={math.degrees(yaw):.2f}deg"
        )


def main() -> None:
    rclpy.init()
    node = PoseFromOdom()
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
