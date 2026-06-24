#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class LidarTfOffset(Node):
    def __init__(self, args):
        super().__init__('lidar_tf_offset')

        self.target_frame = args.target_frame
        self.source_frame = args.source_frame
        self.topic = args.topic
        self.warn_period = max(0.1, args.warn_period)
        self._last_warn = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Vector3Stamped, self.topic, 10)

        publish_rate = args.publish_rate if args.publish_rate > 0.0 else 30.0
        self.timer = self.create_timer(1.0 / publish_rate, self._tick)

        self.get_logger().info(
            f'Publishing TF translation {self.target_frame} -> '
            f'{self.source_frame} on {self.topic}'
        )

    def _warn_throttled(self, message: str):
        now = time.monotonic()
        if now - self._last_warn >= self.warn_period:
            self._last_warn = now
            self.get_logger().warn(message)

    def _tick(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                Time(),
            )
        except TransformException as exc:
            self._warn_throttled(
                f'Waiting for TF {self.target_frame} -> {self.source_frame}: {exc}'
            )
            return

        t = tf_msg.transform.translation
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.vector.x = float(t.x)
        msg.vector.y = float(t.y)
        msg.vector.z = float(t.z)
        self.publisher.publish(msg)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Publish LiDAR TF offset as a Vector3Stamped topic.'
    )
    parser.add_argument('--publish-rate', type=float, default=30.0)
    parser.add_argument('--warn-period', type=float, default=2.0)
    parser.add_argument('--target-frame', default='map')
    parser.add_argument('--source-frame', default='body')
    parser.add_argument('--topic', default='/tf_compare/map_to_body')
    return parser.parse_known_args(argv)


def main(argv=None):
    cli_args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = LidarTfOffset(cli_args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
