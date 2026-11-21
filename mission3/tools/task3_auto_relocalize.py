#!/usr/bin/env python3
"""Automatically search the saved map, then initialize the existing localizer.

No mouse click or manually entered current pose. The car must remain stationary
while a short cloud window is collected and the global candidates are checked.
The default saves candidates only; --initialize enables a single guarded
/initialpose update after checking the live map/localizer and source identity.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/task1_fusion'))
from task1_fusion.global_relocalization import select_candidate, matrix
from task1_fusion.task3_config import sha256
from task1_fusion.corner import rotation_quaternion
from task1_fusion.math_utils import quat_matrix


def write_pcd(path, points):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    header = ('# PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
              'COUNT 1 1 1\nWIDTH '+str(len(points))+'\nHEIGHT 1\n'
              'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS '+str(len(points))+'\nDATA binary\n')
    with Path(path).open('xb') as file:
        file.write(header.encode('ascii'))
        file.write(points.tobytes())


def build_solver(binary):
    source = ROOT/'tools/global_relocalization/search.cpp'
    binary = Path(binary)
    flags = subprocess.check_output(['pkg-config', '--cflags', '--libs', 'pcl_registration',
                                     'pcl_features', 'pcl_io', 'pcl_filters', 'pcl_kdtree'], text=True)
    binary.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(['g++', '-std=c++17', '-O2', str(source), '-o', str(binary),
                    *shlex.split(flags)], check=True)
    return binary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--solver', type=Path)
    parser.add_argument('--initialize', action='store_true')
    parser.add_argument('--iterations', type=int, default=50000)
    parser.add_argument('--data-source', choices=('live', 'synthetic'), default='live')
    args = parser.parse_args()
    if args.out_dir.exists():
        parser.error('output directory already exists; keep the existing evidence')
    if not args.map.is_file() or not 1000 <= args.iterations <= 500000:
        parser.error('missing map or iterations out of range')
    out, map_file = args.out_dir.resolve(), args.map.resolve()
    out.mkdir(parents=True)
    solver = args.solver.resolve() if args.solver else ROOT/'.cache/task3_global_search/search'
    source = ROOT/'tools/global_relocalization/search.cpp'
    checksum_file = solver.with_name(solver.name+'.source_sha256')
    rebuild = (not solver.is_file() or (args.solver is None and
               (not checksum_file.is_file() or checksum_file.read_text().strip() != sha256(source))))
    if rebuild:
        print('首次编译自动搜索工具；不会修改驱动/FAST-LIO/地图。', flush=True)
        build_solver(solver)
        if args.solver is None:
            checksum_file.write_text(sha256(source)+'\n')
    os.environ.setdefault('ROS_DOMAIN_ID', '0')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.parameter import parameter_value_to_python
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rclpy.init(args=[])
    node = Node('task3_automatic_map_initializer')
    clouds, stamps, positions = [], [], []
    last_odom, last_receipt = None, None
    source_gids, capture_error = None, ''
    proc = None

    def graph():
        pubs = {topic: node.get_publishers_info_by_topic(topic)
                for topic in ('/Odometry', '/cloud_registered')}
        if any(len(values) != 1 for values in pubs.values()):
            raise ValueError('need exactly one FAST-LIO /Odometry and /cloud_registered publisher')
        return {key: list(values[0].endpoint_gid) for key, values in pubs.items()}

    def odom(msg):
        nonlocal last_odom, last_receipt, capture_error
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds-stamp)*1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if (msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body'
                or stamp <= 0 or not -.05 <= age <= .5 or not np.isfinite(values).all()):
            return
        if last_odom is not None:
            old = last_odom.pose.pose.position
            if np.linalg.norm(np.array(values[:3])-[old.x, old.y, old.z]) > .15:
                capture_error = 'car moved or LIO reset during automatic search; no initialization sent'
        last_odom, last_receipt = msg, time.monotonic()
        positions.append(values[:3])

    def cloud(msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds-stamp)*1e-9
        if (len(clouds) >= 10 or msg.header.frame_id != 'camera_init'
                or not -.05 <= age <= .5 or (stamps and stamp <= stamps[-1])):
            return
        values = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if len(values) < 200:
            return
        clouds.append(np.asarray(values, dtype=np.float32).reshape(-1, 3).copy())
        stamps.append(stamp)

    node.create_subscription(Odometry, '/Odometry', odom, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/cloud_registered', cloud, qos_profile_sensor_data)
    try:
        client = AsyncParameterClient(node, '/map_localizer')
        if not client.wait_for_services(timeout_sec=5.):
            raise ValueError('map_localizer parameter service is unavailable; keep map localizer running')
        names = ['map_path', 'auto_initialize_identity', 'map_frame', 'body_frame']
        future = client.get_parameters(names)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done():
            raise ValueError('map identity query timed out')
        params = dict(zip(names, [parameter_value_to_python(v) for v in future.result().values]))
        if (Path(params['map_path']).resolve() != map_file or params['auto_initialize_identity'] is not False
                or params['map_frame'] != 'map' or params['body_frame'] != 'body'):
            raise ValueError('map-localizer identity does not match the selected saved map')
        source_gids = graph()
        map_digest = sha256(map_file)
        print('保持车静止：自动采集当前扫描，无需在RViz指定位置。', flush=True)
        deadline = time.monotonic()+15.
        while len(clouds) < 10 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if len(clouds) < 10 or last_odom is None:
            raise ValueError('not enough fresh scans/odometry; no initialization sent')
        if np.max(np.ptp(positions, axis=0)) > .03:
            raise ValueError('car is not stationary enough; no initialization sent')
        write_pcd(out/'source.pcd', np.concatenate(clouds[:6]))
        write_pcd(out/'heldout.pcd', np.concatenate(clouds[6:]))
        metadata = {'created_at_utc': datetime.now(timezone.utc).isoformat(),
                    'data_kind': 'live_lio_cloud_capture' if args.data_source == 'live' else 'synthetic_ros_capture',
                    'map': str(map_file), 'map_sha256': map_digest,
                    'cloud_frame': 'camera_init', 'cloud_source_stamps_ns': stamps,
                    'source_publisher_gids': source_gids, 'physical_accuracy_verified': False}
        (out/'capture.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
        print('正在整图搜索并检查独立扫描；请保持车静止。', flush=True)
        with (out/'solver.log').open('w') as log:
            proc = subprocess.Popen([str(solver), str(map_file), str(out/'source.pcd'),
                                     str(out/'heldout.pcd'), str(out/'candidates.json'), str(args.iterations)],
                                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+240.
            while proc.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if capture_error or graph() != source_gids:
                    raise ValueError(capture_error or 'FAST-LIO restarted during search')
            if proc.poll() is None:
                raise ValueError('global search exceeded 240 seconds; no initialization sent')
        if proc.returncode != 0:
            raise ValueError('global search failed; see '+str(out/'solver.log'))
        decision = select_candidate(json.loads((out/'candidates.json').read_text()))
        decision.update(initialization_sent=False, physical_accuracy_verified=False)
        (out/'decision.json').write_text(json.dumps(decision, ensure_ascii=False, indent=2)+'\n')
        if not decision['accepted']:
            print('尚未得到可信的唯一位置：'+decision['reason']+'。未向定位程序发送初值。', flush=True)
            return 2
        print('候选通过软件几何检查；仍需现有地图定位精配准。', flush=True)
        if not args.initialize:
            print('本次只保存结果，未改变实时定位。结果：'+str(out/'decision.json'))
            return 0
        if (capture_error or last_receipt is None or time.monotonic()-last_receipt > .5
                or graph() != source_gids or sha256(map_file) != map_digest):
            raise ValueError('source/map changed or became stale; no initialization sent')
        pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        deadline = time.monotonic()+3.
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if pub.get_subscription_count() == 0:
            raise ValueError('no initialpose subscriber; no initialization sent')
        if last_receipt is None or time.monotonic()-last_receipt > .5 or graph() != source_gids:
            raise ValueError('source became stale during discovery; no initialization sent')
        # Apply the SEARCHED map->camera_init to a CURRENT source-stamped body
        # pose. Sending the old capture pose with a new stamp would be wrong.
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = last_odom.header.stamp
        msg.header.frame_id = 'map'
        body = last_odom.pose.pose
        S = np.eye(4)
        S[:3, :3] = quat_matrix([body.orientation.x, body.orientation.y, body.orientation.z, body.orientation.w])
        S[:3, 3] = [body.position.x, body.position.y, body.position.z]
        T = matrix(decision['best']) @ S
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        p.x, p.y, p.z = map(float, T[:3, 3])
        q.x, q.y, q.z, q.w = map(float, rotation_quaternion(T[:3, :3]))
        # Coarse uncertainty; not a claimed precision measurement.
        msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[14] = .25
        msg.pose.covariance[35] = math.radians(15.)**2
        age = (node.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))*1e-9
        if not -.05 <= age <= .5:
            raise ValueError('initial pose aged while waiting for subscriber; rerun without resetting origin')
        pub.publish(msg)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=.02)
        decision.update(initialization_sent=True, initial_body_pose_map=T.tolist(),
                        initial_source_stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
        (out/'decision.json').write_text(json.dumps(decision, ensure_ascii=False, indent=2)+'\n')
        print('已自动发送定位初值，等待地图定位精配准；正式原点尚未绑定，不会自动清零。')
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.CalledProcessError) as exc:
        (out/'failure.json').write_text(json.dumps({'error': str(exc), 'initialization_sent': False,
            'physical_accuracy_verified': False}, ensure_ascii=False, indent=2)+'\n')
        print('自动定位未完成：'+str(exc), file=sys.stderr)
        return 2
    finally:
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=3.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
