#!/usr/bin/env python3
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class PoseFromTf(Node):
    """Publish the robot business pose from a TF lookup."""

    def __init__(self) -> None:
        super().__init__("robot_pose_from_tf")

        self.declare_parameter("target_frame", "robot_origin")
        self.declare_parameter("source_frame", "robot_base_highrate")
        self.declare_parameter("z_source_frame", "robot_base")
        self.declare_parameter("output_pose_topic", "/robot/pose")
        self.declare_parameter("output_pose_alias_topic", "/lidar/pose")
        self.declare_parameter("output_point_topic", "/lidar/point")
        self.declare_parameter("output_odom_topic", "/lidar/odom")
        numeric_parameter = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("publish_rate", 50.0, numeric_parameter)
        self.declare_parameter("warn_period_sec", 2.0)
        self.declare_parameter("max_tf_age_sec", 1.0, numeric_parameter)

        self.target_frame = self.get_parameter("target_frame").value
        self.source_frame = self.get_parameter("source_frame").value
        self.z_source_frame = self.get_parameter("z_source_frame").value
        self.output_pose_topic = self.get_parameter("output_pose_topic").value
        self.output_pose_alias_topic = self.get_parameter(
            "output_pose_alias_topic"
        ).value
        self.output_point_topic = self.get_parameter("output_point_topic").value
        self.output_odom_topic = self.get_parameter("output_odom_topic").value
        self.warn_period_sec = float(self.get_parameter("warn_period_sec").value)
        self.max_tf_age_sec = float(self.get_parameter("max_tf_age_sec").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0.0:
            publish_rate = 50.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.pose_alias_pub = (
            self.create_publisher(PoseStamped, self.output_pose_alias_topic, 10)
            if self.output_pose_alias_topic
            else None
        )
        self.point_pub = self.create_publisher(PointStamped, self.output_point_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)

        self.last_warn = 0.0
        self.timer = self.create_timer(1.0 / publish_rate, self._tick)
        self.get_logger().info(
            f"publishing {self.target_frame} -> {self.source_frame} as "
            f"{self.output_pose_topic}"
            f"{f' and {self.output_pose_alias_topic}' if self.output_pose_alias_topic else ''}"
        )

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warn >= self.warn_period_sec:
            self.last_warn = now
            self.get_logger().warn(message)

    def _lookup(self, source_frame: str):
        return self.tf_buffer.lookup_transform(
            self.target_frame,
            source_frame,
            Time(),
        )

    def _tf_age_ok(self, tf_msg) -> bool:
        stamp = Time.from_msg(tf_msg.header.stamp)
        if stamp.nanoseconds == 0 or self.max_tf_age_sec <= 0.0:
            return True
        age_sec = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age_sec > self.max_tf_age_sec:
            self._warn_throttled(
                f"TF {self.target_frame}->{self.source_frame} is stale: "
                f"age={age_sec:.3f}s"
            )
            return False
        return True

    def _tick(self) -> None:
        try:
            high_tf = self._lookup(self.source_frame)
            z_tf = self._lookup(self.z_source_frame) if self.z_source_frame else None
        except TransformException as exc:
            self._warn_throttled(
                f"Waiting for TF {self.target_frame} -> {self.source_frame}: {exc}"
            )
            return

        if not self._tf_age_ok(high_tf):
            return

        stamp = self.get_clock().now().to_msg()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.target_frame
        pose_msg.pose.position.x = high_tf.transform.translation.x
        pose_msg.pose.position.y = high_tf.transform.translation.y
        pose_msg.pose.position.z = (
            z_tf.transform.translation.z
            if z_tf is not None
            else high_tf.transform.translation.z
        )
        pose_msg.pose.orientation = high_tf.transform.rotation
        self.pose_pub.publish(pose_msg)
        if self.pose_alias_pub:
            self.pose_alias_pub.publish(pose_msg)

        point_msg = PointStamped()
        point_msg.header = pose_msg.header
        point_msg.point = pose_msg.pose.position
        self.point_pub.publish(point_msg)

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = self.source_frame
        odom_msg.pose.pose = pose_msg.pose
        self.odom_pub.publish(odom_msg)


def main() -> None:
    rclpy.init()
    node = PoseFromTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
