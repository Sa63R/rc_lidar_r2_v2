#!/usr/bin/env python3
"""Example for the vision team: consume the agreed CORNER, in m and rad."""
import json
import math
import time
import argparse
from collections import Counter
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_corner_subscriber_example')
        self.latest = None
        self.receipt = self.last_log = 0.
        self.unique_samples = 0
        self.invalid_reasons = Counter()
        self.reference_ids = set()
        self.last_identity = None
        self.create_subscription(String, '/vision/state', self.receive, 20)
        self.create_timer(.1, self.check_timeout)

    def receive(self, msg):
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict):
                raise ValueError('vision state must be an object')
            data = state.get('data')
            if (state.get('schema_version') != 1 or not state.get('valid')
                    or state.get('reference_point') != 'corner' or data is None
                    or state.get('position_unit') != 'm' or state.get('yaw_unit') != 'rad'):
                self.latest = None
                self.invalid_reasons[state.get('reason', 'unexpected_contract')] += 1
                if time.monotonic()-self.last_log > 1.:
                    self.last_log = time.monotonic()
                    self.get_logger().info(f'No usable coordinates: {state.get("reason", "invalid_contract")}')
                return
            age = (self.get_clock().now().nanoseconds-int(data['stamp_ns']))*1e-9
            if not -.05 <= age <= .5:
                self.latest = None
                return
            x, y, yaw = (float(data[k]) for k in ('x', 'y', 'yaw'))
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError('nonfinite coordinates')
            self.latest, self.receipt = data, time.monotonic()
            identity = (state.get('reference_id', ''), data['frame_id'], int(data['stamp_ns']))
            if identity != self.last_identity:
                if self.last_identity is not None and identity[:2] != self.last_identity[:2]:
                    self.get_logger().warning('Reference changed: clear trajectory/offsets from the previous origin.')
                self.last_identity = identity
                self.unique_samples += 1
            self.reference_ids.add(state.get('reference_id', ''))
            # Apply the vision team's own corner -> center / camera transform here.
            if self.receipt-self.last_log > 1.:
                self.last_log = self.receipt
                self.get_logger().info(f'corner: x={x:.4f} m, y={y:.4f} m, yaw={yaw:.4f} rad; '
                                       f'frame={data["frame_id"]}, stamp_ns={data["stamp_ns"]}; '
                                       f'source={state.get("data_source", "unknown")}, '
                                       f'physical_accuracy_verified={state.get("physical_accuracy_verified", False)}')
        except (ValueError, TypeError, KeyError):
            self.latest = None

    def check_timeout(self):
        if self.latest is not None:
            age = (self.get_clock().now().nanoseconds-self.latest['stamp_ns'])*1e-9
            if time.monotonic()-self.receipt > .5 or not -.05 <= age <= .5:
                self.latest = None
                self.get_logger().warning('Corner coordinates expired; do not reuse the last pose.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=0, help='0: until Ctrl+C')
    parser.add_argument('--report', type=Path, help='new JSON file, refuses overwrite')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('--seconds must be finite and nonnegative')
    if args.report and args.report.exists():
        parser.error('report already exists; refusing overwrite')
    rclpy.init(args=[])
    node = VisionSubscriber()
    deadline = time.monotonic()+args.seconds if args.seconds else math.inf
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps({
                'valid_unique_samples': node.unique_samples,
                'invalid_heartbeat_reasons': dict(node.invalid_reasons),
                'reference_ids': sorted(node.reference_ids),
                'scope': '本机示例订阅接收记录；不代表视觉业务程序已接入，不验证物理精度。',
            }, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.unique_samples else 2


if __name__ == '__main__':
    raise SystemExit(main())
