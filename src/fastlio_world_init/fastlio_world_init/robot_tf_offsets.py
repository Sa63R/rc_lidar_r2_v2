#!/usr/bin/env python3

import argparse
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


@dataclass
class TfOutput:
    target_frame: str
    source_frame: str
    topic: str
    publisher: object


class RobotTfOffsets(Node):
    def __init__(self, args):
        super().__init__('robot_tf_offsets')

        self.warn_period = max(0.1, args.warn_period)
        self._last_warn = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.outputs = [
            TfOutput(
                args.target_frame,
                args.base_link_frame,
                args.low_topic,
                self.create_publisher(Vector3Stamped, args.low_topic, 10),
            ),
            TfOutput(
                args.target_frame,
                args.base_link_highrate_frame,
                args.highrate_topic,
                self.create_publisher(Vector3Stamped, args.highrate_topic, 10),
            ),
        ]

        publish_rate = args.publish_rate if args.publish_rate > 0.0 else 30.0
        self.timer = self.create_timer(1.0 / publish_rate, self._tick)

        for output in self.outputs:
            self.get_logger().info(
                f'Publishing TF translation {output.target_frame} -> '
                f'{output.source_frame} on {output.topic}'
            )

    def _warn_throttled(self, key: str, message: str):
        now = time.monotonic()
        if now - self._last_warn.get(key, 0.0) >= self.warn_period:
            self._last_warn[key] = now
            self.get_logger().warn(message)

    def _tick(self):
        for output in self.outputs:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    output.target_frame,
                    output.source_frame,
                    Time(),
                )
            except TransformException as exc:
                self._warn_throttled(
                    output.topic,
                    f'Waiting for TF {output.target_frame} -> '
                    f'{output.source_frame}: {exc}',
                )
                continue

            t = tf_msg.transform.translation
            msg = Vector3Stamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = output.target_frame
            msg.vector.x = float(t.x)
            msg.vector.y = float(t.y)
            msg.vector.z = float(t.z)
            output.publisher.publish(msg)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Publish robot center TF offsets as Vector3Stamped topics.'
    )
    parser.add_argument('--publish-rate', type=float, default=30.0)
    parser.add_argument('--warn-period', type=float, default=2.0)
    parser.add_argument('--target-frame', default='init')
    parser.add_argument('--base-link-frame', default='base_link')
    parser.add_argument('--base-link-highrate-frame', default='base_link_highrate')
    parser.add_argument(
        '--low-topic',
        default='/tf_compare/robot_init_to_base_link',
    )
    parser.add_argument(
        '--highrate-topic',
        default='/tf_compare/robot_init_to_base_link_highrate',
    )
    return parser.parse_known_args(argv)


def main(argv=None):
    cli_args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = RobotTfOffsets(cli_args)
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
