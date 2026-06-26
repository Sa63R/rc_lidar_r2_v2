#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class CanPoseMonitor(Node):
    def __init__(self, topic: str, hz: float):
        super().__init__("can_pose_monitor")
        self.topic = topic
        self.min_interval = 0.0 if hz <= 0.0 else 1.0 / hz
        self.last_print_time = 0.0
        self.last_msg_time = None
        self.count = 0
        self.create_subscription(Float32MultiArray, topic, self.callback, 20)
        print(f"[can_monitor] listening: {topic}", flush=True)

    def callback(self, msg: Float32MultiArray):
        now = time.monotonic()
        if self.min_interval > 0.0 and now - self.last_print_time < self.min_interval:
            return
        self.last_print_time = now

        data = list(msg.data)
        if len(data) < 4:
            print(f"[can_monitor] short message: {data}", flush=True)
            return

        self.count += 1
        dt = None
        if self.last_msg_time is not None:
            dt = now - self.last_msg_time
        self.last_msg_time = now

        x, y, z, yaw = data[:4]
        xy_sent = bool(data[4] > 0.5) if len(data) > 4 else True
        hz_text = "--"
        if dt and dt > 0.0:
            hz_text = f"{1.0 / dt:5.1f}"

        print(
            "CAN "
            f"#{self.count:06d} "
            f"x={x:+.4f}m "
            f"y={y:+.4f}m "
            f"z={z:+.4f}m "
            f"yaw={yaw:+.4f}rad "
            f"yaw={math.degrees(yaw):+7.2f}deg "
            f"xy_sent={int(xy_sent)} "
            f"print_hz={hz_text}",
            flush=True,
        )


def main():
    parser = argparse.ArgumentParser(description="Print pose values prepared by rc_serial_driver for CAN.")
    parser.add_argument("--topic", default="/rc_serial_driver/can_pose_debug")
    parser.add_argument("--hz", type=float, default=10.0, help="Max print rate. Use <=0 to print every message.")
    args = parser.parse_args()

    rclpy.init()
    node = CanPoseMonitor(args.topic, args.hz)
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
