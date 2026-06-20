#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .pose_math import rpy_from_quat


class XYYawPrinter(Node):
    def __init__(self) -> None:
        super().__init__("print_xy_yaw")
        self.declare_parameter("topic", "/lidar/pose")
        self.declare_parameter("messages", 0)
        self.declare_parameter("degrees", True)
        self.declare_parameter("yaw_offset_deg", 0.0)
        self.topic = self.get_parameter("topic").value
        self.messages = int(self.get_parameter("messages").value)
        self.use_degrees = bool(self.get_parameter("degrees").value)
        self.yaw_offset_rad = math.radians(
            float(self.get_parameter("yaw_offset_deg").value)
        )
        self.count = 0
        self.done = False
        self.subscription = self.create_subscription(
            PoseStamped, self.topic, self.callback, 10
        )
        self.get_logger().info(f"listening to {self.topic}")

    def callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        _, _, yaw = rpy_from_quat(msg.pose.orientation)
        yaw = self.normalize_angle(yaw + self.yaw_offset_rad)
        if self.use_degrees:
            yaw_value = math.degrees(yaw)
            yaw_text = f"{yaw_value:.2f}deg"
        else:
            yaw_text = f"{yaw:.6f}rad"

        print(
            f"frame={msg.header.frame_id} "
            f"x={p.x:.3f} y={p.y:.3f} yaw={yaw_text}",
            flush=True,
        )
        self.count += 1
        if self.messages > 0 and self.count >= self.messages:
            self.done = True

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main() -> None:
    rclpy.init()
    node = XYYawPrinter()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
