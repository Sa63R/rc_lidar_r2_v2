#!/usr/bin/env python3
"""A tiny differential-drive simulator for Nav2 bringup tests.

The node subscribes to cmd_vel, integrates a planar pose, then publishes
nav_msgs/Odometry and odom -> base_link TF. It is intentionally simple: no
physics, no wheel slip, no collision model.
"""

import math

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class FakeDiffDrive(Node):
    def __init__(self):
        super().__init__("fake_diff_drive")

        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.publish_rate = float(self.declare_parameter("publish_rate", 50.0).value)
        self.cmd_vel_timeout = float(
            self.declare_parameter("cmd_vel_timeout", 0.5).value
        )
        self.max_linear_speed = float(
            self.declare_parameter("max_linear_speed", 0.6).value
        )
        self.max_angular_speed = float(
            self.declare_parameter("max_angular_speed", 1.5).value
        )

        self.x = float(self.declare_parameter("start_x", 10.0).value)
        self.y = float(self.declare_parameter("start_y", 0.0).value)
        self.yaw = float(self.declare_parameter("start_yaw", 0.0).value)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            10,
        )

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self.update)
        self.get_logger().info(
            "fake diff drive ready: cmd=%s odom=%s start=(%.2f, %.2f, %.2f)"
            % (self.cmd_vel_topic, self.odom_topic, self.x, self.y, self.yaw)
        )

    def cmd_callback(self, msg):
        self.linear_x = clamp(
            float(msg.linear.x), -self.max_linear_speed, self.max_linear_speed
        )
        self.angular_z = clamp(
            float(msg.angular.z), -self.max_angular_speed, self.max_angular_speed
        )
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1.0e-9
        self.last_update_time = now
        if dt <= 0.0:
            self.publish(now, 0.0, 0.0)
            return

        cmd_age = (now - self.last_cmd_time).nanoseconds * 1.0e-9
        if cmd_age > self.cmd_vel_timeout:
            linear_x = 0.0
            angular_z = 0.0
        else:
            linear_x = self.linear_x
            angular_z = self.angular_z

        self.x += linear_x * math.cos(self.yaw) * dt
        self.y += linear_x * math.sin(self.yaw) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + angular_z * dt),
            math.cos(self.yaw + angular_z * dt),
        )
        self.publish(now, linear_x, angular_z)

    def publish(self, stamp, linear_x, angular_z):
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.angular.z = angular_z
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = FakeDiffDrive()
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
