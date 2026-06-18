#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LidarXYZRPrinter(Node):
    def __init__(self, topic: str, hz: float, max_points: int):
        super().__init__('lidar_xyzr_printer')
        self._topic = topic
        self._min_interval = 1.0 / hz if hz > 0 else 0.0
        self._last_print = 0.0
        self._latest_msg = None
        self._max_points = max_points

        self.create_subscription(PointCloud2, topic, self._on_cloud, 10)
        self.create_timer(0.02, self._tick)  # 50 Hz timer
        self.get_logger().info(
            f"Listening on '{topic}', printing at <= {hz} Hz, max_points={max_points}")

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._latest_msg = msg

    def _pick_r_field(self, msg: PointCloud2) -> str | None:
        field_names = {f.name for f in msg.fields}
        for name in ("intensity", "reflectivity", "reflectance", "r"):
            if name in field_names:
                return name
        return None

    def _tick(self) -> None:
        if self._latest_msg is None:
            return
        now = time.time()
        if self._min_interval > 0 and (now - self._last_print) < self._min_interval:
            return
        self._last_print = now

        msg = self._latest_msg
        r_field = self._pick_r_field(msg)
        fields = ["x", "y", "z"] + ([r_field] if r_field else [])

        try:
            points_iter = point_cloud2.read_points(msg, field_names=fields, skip_nans=True)
        except Exception as exc:
            self.get_logger().warn(f"Failed to read points: {exc}")
            return

        header = (
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
            f"frame_id={msg.header.frame_id} width={msg.width} height={msg.height} "
            f"fields={fields}"
        )
        sys.stdout.write(header + "\n")

        count = 0
        for pt in points_iter:
            x, y, z = pt[0], pt[1], pt[2]
            r_val = pt[3] if r_field and len(pt) > 3 else None
            if r_val is None:
                sys.stdout.write(f"  [{count}] x={x:.3f} y={y:.3f} z={z:.3f} r=N/A\n")
            else:
                sys.stdout.write(f"  [{count}] x={x:.3f} y={y:.3f} z={z:.3f} r={r_val:.3f}\n")
            count += 1
            if count >= self._max_points:
                break

        if count == 0:
            sys.stdout.write("  (no valid points)\n")

        sys.stdout.flush()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="livox/lidar", help="PointCloud2 topic")
    parser.add_argument("--hz", type=float, default=10.0, help="Print rate limit")
    parser.add_argument("--max-points", type=int, default=5, help="Max points to print per message")
    args = parser.parse_args()

    rclpy.init()
    node = LidarXYZRPrinter(args.topic, args.hz, args.max_points)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
