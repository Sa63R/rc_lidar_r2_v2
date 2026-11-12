#!/usr/bin/env python3
"""Isolated SYNTHETIC stationary input -> real corner/vision nodes -> desktop.

This is a software interface exercise, never real-car precision evidence.
"""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/task1_fusion'))
from task1_fusion.corner_node import CornerNode
from task1_fusion.vision_node import VisionNode
from task1_fusion.task3_config import load_config, relative_start_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-gui', action='store_true')
    parser.add_argument('--seconds', type=float, default=0, help='0: until window close or Ctrl+C')
    parser.add_argument('--drop-after', type=float, default=0, help='stop synthetic input after N seconds')
    parser.add_argument('--report', type=Path, help='new JSON report')
    args = parser.parse_args()
    if any(not math.isfinite(v) or v < 0 for v in (args.seconds, args.drop_after)):
        parser.error('durations must be finite and nonnegative')
    if args.report and args.report.exists():
        parser.error('report already exists; refusing overwrite')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        parser.error('run through scripts/task3_demo.sh in its isolated ROS domain')
    base = load_config(ROOT/'config/car_task3_local.yaml')
    config = relative_start_config(base, [0., 0., 0.], [0., 0., 0., 1.])
    cp = config['task3_corner']['ros__parameters']
    cp['input_topic'] = '/task3_demo/input'
    reference = f'synthetic_stationary_{datetime.now():%Y%m%d_%H%M%S}'
    rclpy.init(args=[])
    probe = Node('task3_stationary_demo')
    corner = vision = executor = gui = None
    sent = 0
    received_valid = 0
    invalid_after_valid = 0
    latest = None
    latest_log = 0.
    begun = time.monotonic()

    def publish():
        nonlocal sent
        if args.drop_after and time.monotonic()-begun >= args.drop_after:
            return
        msg = Odometry()
        msg.header.stamp = probe.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'camera_init', 'body'
        msg.pose.pose.orientation.w = 1.
        publisher.publish(msg)
        sent += 1

    def receive(msg):
        nonlocal received_valid, invalid_after_valid, latest, latest_log
        latest = json.loads(msg.data)
        if latest['valid']:
            received_valid += 1
        elif received_valid:
            invalid_after_valid += 1
            assert latest['data'] is None
        if time.monotonic()-latest_log > 1:
            latest_log = time.monotonic()
            print(json.dumps(latest, ensure_ascii=False), flush=True)

    try:
        # Refuse to mix with another demo/stack already running in this domain.
        for _ in range(10):
            rclpy.spin_once(probe, timeout_sec=.05)
        if probe.count_publishers('/vision/state'):
            parser.error('vision interface already exists in this demo domain; stop the previous demo first')
        corner = CornerNode(parameter_overrides=[Parameter(k, value=v) for k, v in cp.items()])
        vision = VisionNode(parameter_overrides=[Parameter(k, value=v) for k, v in {
            'input_parent_frame': 'task3_start', 'allow_unverified_extrinsics': True,
            'reference_id': reference, 'data_source': 'synthetic_stationary',
        }.items()])
        publisher = probe.create_publisher(Odometry, '/task3_demo/input', 10)
        probe.create_timer(.1, publish)
        probe.create_subscription(String, '/vision/state', receive, 20)
        executor = SingleThreadedExecutor()
        for node in (probe, corner, vision):
            executor.add_node(node)
        if not args.no_gui:
            gui = subprocess.Popen(['ros2', 'run', 'task1_fusion', 'vision_ui', '--monitor-only'])
        print(f'离线演示：合成静止数据，ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]}；'
              '不会启动或连接实车。Ctrl+C或关闭演示窗口结束。', flush=True)
        while (not args.seconds or time.monotonic()-begun < args.seconds):
            executor.spin_once(timeout_sec=.05)
            if gui is not None and gui.poll() is not None:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if gui is not None and gui.poll() is None:
            gui.send_signal(signal.SIGINT)
            try:
                gui.wait(timeout=5)
            except subprocess.TimeoutExpired:
                gui.kill()
                gui.wait()
        if executor is not None:
            executor.shutdown()
        for node in (vision, corner, probe):
            if node is not None:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    report = {'data_kind': 'synthetic_stationary', 'ros_domain_id': os.environ['ROS_DOMAIN_ID'],
              'synthetic_source_samples': sent, 'valid_heartbeats': received_valid,
              'invalid_heartbeats_after_valid': invalid_after_valid,
              'last_state': latest, 'physical_accuracy_verified': False,
              'scope': '仅验证软件发布/接收/界面/合成断流，不代表实车定位或视觉业务联调完成。'}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    if not received_valid:
        return 2
    if args.drop_after and args.seconds > args.drop_after+1:
        return 0 if invalid_after_valid and latest['data'] is None else 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
