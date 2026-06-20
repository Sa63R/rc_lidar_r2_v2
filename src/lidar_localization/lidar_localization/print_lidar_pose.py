#!/usr/bin/env python3
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .pose_math import rpy_from_quat


class PosePrinter(Node):
    def __init__(self) -> None:
        super().__init__("print_lidar_pose")
        self.declare_parameter("topic", "/lidar/pose")
        self.declare_parameter("messages", 0)
        self.topic = self.get_parameter("topic").value
        self.messages = int(self.get_parameter("messages").value)
        self.count = 0
        self.done = False
        self.subscription = self.create_subscription(
            PoseStamped, self.topic, self.callback, 10
        )
        self.get_logger().info(f"listening to {self.topic}")

    def callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        roll, pitch, yaw = rpy_from_quat(msg.pose.orientation)
        print(
            f"frame={msg.header.frame_id} "
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
            f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} "
            f"roll={math.degrees(roll):.2f}deg "
            f"pitch={math.degrees(pitch):.2f}deg "
            f"yaw={math.degrees(yaw):.2f}deg",
            flush=True,
        )
        self.count += 1
        if self.messages > 0 and self.count >= self.messages:
            self.done = True


def main() -> None:
    rclpy.init()
    node = PosePrinter()
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
