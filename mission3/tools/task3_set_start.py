#!/usr/bin/env python3
"""Capture one fresh source pose and save a new fixed corner-start frame."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/task1_fusion'))
from task1_fusion.task3_config import load_config, prepare, relative_start_config, sha256
from task1_fusion.task3_quality import StableStartWindow


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-config', type=Path, default=ROOT/'config/car_task3_local.yaml')
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--launch', action='store_true', help='replace this process with Task-3 launch')
    parser.add_argument('--active-config-link', type=Path, default=ROOT/'config/car_task3_start.yaml')
    parser.add_argument('--launch-arg', action='append', default=[])
    args = parser.parse_args()
    if args.out_dir.exists():
        parser.error('output directory already exists; refusing overwrite')
    link = args.active_config_link.resolve() if not args.active_config_link.is_symlink() else args.active_config_link.absolute()
    if args.launch and link.exists() and not link.is_symlink():
        parser.error('active configuration is a regular file; refusing overwrite')
    base = load_config(args.base_config)
    p, projector = prepare(base['task3_corner']['ros__parameters'])
    rclpy.init(args=[])
    node = rclpy.create_node('task3_start_frame_capture')
    received = []
    window = StableStartWindow(projector, max_gap_sec=p['input_timeout_sec'])

    def receive(msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds-stamp)*1e-9
        if (msg.header.frame_id == p['input_parent_frame']
                and msg.child_frame_id == p['input_child_frame']
                and stamp > 0 and -p['future_tolerance_sec'] <= age <= p['input_timeout_sec']):
            pos, q = msg.pose.pose.position, msg.pose.pose.orientation
            try:
                projector.pose([pos.x, pos.y, pos.z], [q.x, q.y, q.z, q.w], msg.pose.covariance)
            except ValueError:
                return
            if window.add(stamp, [pos.x, pos.y, pos.z], [q.x, q.y, q.z, q.w]):
                received[:] = [msg]

    node.create_subscription(Odometry, p['input_topic'], receive, qos_profile_sensor_data)
    try:
        deadline = time.monotonic()+8
        print('请保持车静止：采集连续约2秒定位样本建立起点。', flush=True)
        estimate = None
        while estimate is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            estimate = window.estimate()
        if estimate is None:
            parser.error('no fresh low-variation 2-second source window; keep car still and retry; no origin was created')
        msg = received[-1]
        xyz, quat = estimate['source_xyz_m'], estimate['source_quaternion_xyzw']
        result = relative_start_config(base, xyz, quat)
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        reference_id = f'task3_start_{stamp}'
        result.setdefault('task3_vision', {}).setdefault('ros__parameters', {})['reference_id'] = reference_id
        args.out_dir.mkdir(parents=True, exist_ok=False)
        output = args.out_dir/'task3_corner.yaml'
        output.write_text('# Fixed initial-corner XY frame; Z is not ground height.\n'
                          +yaml.safe_dump(result, sort_keys=False, allow_unicode=True), encoding='utf-8')
        samples_file = args.out_dir/'start_samples.json'
        samples_file.write_text(json.dumps({
            'data_source': result['task3_vision']['ros__parameters'].get('data_source', 'live'),
            'reference_id': reference_id, 'source_parent_frame': msg.header.frame_id,
            'samples': [{'stamp_ns': row[0], 'source_xyz_m': row[1].tolist(),
                         'source_quaternion_xyzw': row[2].tolist()}
                        for row in window.rows],
            'physical_accuracy_verified': False,
        }, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        report = {'created_at_utc': datetime.now(timezone.utc).isoformat(),
                  'source_stamp_ns': msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                  'source_parent_frame': msg.header.frame_id,
                  'source_xyz_m': xyz, 'source_quaternion_xyzw': quat,
                  'source_pose_aggregation': estimate['aggregation'],
                  'source_samples_file': samples_file.name,
                  'source_samples_sha256': sha256(samples_file),
                  'config_sha256': sha256(output), 'output_frame': 'task3_start',
                  'reference_id': reference_id,
                  'allow_unverified_extrinsics': result['task3_vision']['ros__parameters'].get(
                      'allow_unverified_extrinsics', False),
                  'data_source': result['task3_vision']['ros__parameters'].get('data_source', 'live'),
                  'xy_origin': 'initial corner', 'positive_x': 'initial diagram +X projected horizontally',
                  'z_reference': 'unchanged input-world height; deferred, not ground',
                  'calibration_confirmed': p['calibration_confirmed'],
                  'physical_accuracy_verified': False,
                  'scope': '一次初始化后固定坐标轴；FAST-LIO重启或断线重初始化后须重新建起点。'}
        (args.out_dir/'start_reference.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(f'已保存角点起始系：{output.resolve()}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if args.launch:
        link.parent.mkdir(parents=True, exist_ok=True)
        temporary_link = link.with_name(f'.car_task3_start_{os.getpid()}.yaml')
        temporary_link.symlink_to(output.resolve())
        os.replace(temporary_link, link)
        os.execvp('ros2', ['ros2', 'launch', 'task1_fusion', 'task3.launch.py',
                          'input_mode:=lidar', f'corner_config:={output.resolve()}', *args.launch_arg])
    return 0


if __name__ == '__main__':
    sys.exit(main())
