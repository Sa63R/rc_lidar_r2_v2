#!/usr/bin/env python3
"""One-time formal origin binding and daily fixed-map corner output.

Requires the existing map_localizer, initialized in the SAME saved map. This
entry never starts hardware, changes the PCD, or reuses a raw-LIO boot origin.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/task1_fusion'))
from task1_fusion.task3_config import load_config, prepare, sha256
from task1_fusion.task3_fixed_map import MapGate, bind_origin, validate


def ros_watch_class():
    from rclpy.node import Node
    from rclpy.parameter_client import AsyncParameterClient
    from std_msgs.msg import Float32MultiArray
    from rclpy.qos import qos_profile_sensor_data

    class MapWatch(Node):
        def __init__(self, p, meta, node_name='task3_fixed_map_watch'):
            super().__init__(node_name)
            self.p, self.meta = p, meta
            self.interface_started = False
            self.gate = MapGate(p)
            self.future = None
            self.parameter_names = ('map_path', 'map_frame', 'body_frame',
                                    'output_odom_topic', 'output_quality_topic',
                                    'auto_initialize_identity')
            self.client = AsyncParameterClient(self, meta['localizer_node'])
            self.create_subscription(Float32MultiArray, meta['quality_topic'],
                                     self.quality, qos_profile_sensor_data)
            self.create_timer(1., self.check_identity)

        def graph_ok(self):
            if self.interface_started and any(self.count_publishers(topic) != 1 for topic in (
                    '/vision/state', self.p['output_prefix']+'/odom')):
                return False
            for topic in (self.p['input_topic'], self.meta['quality_topic']):
                publishers = self.get_publishers_info_by_topic(topic)
                if len(publishers) != 1:
                    return False
                pub = publishers[0]
                full_name = pub.node_namespace.rstrip('/')+'/'+pub.node_name
                if full_name != self.meta['localizer_node']:
                    return False
            return True

        def quality(self, msg):
            self.gate.quality(msg.data if self.graph_ok() else [])

        def ready(self):
            return self.gate.ready(self.graph_ok())

        def check_identity(self):
            if self.future is not None and not self.future.done():
                return  # Lease expires automatically if the service hangs.
            if not self.client.services_are_ready():
                self.gate.identity({})
                return
            self.future = self.client.get_parameters(list(self.parameter_names))
            self.future.add_done_callback(self.got_identity)

        def got_identity(self, future):
            from rclpy.parameter import parameter_value_to_python
            try:
                values = [parameter_value_to_python(v) for v in future.result().values]
                self.gate.identity(dict(zip(self.parameter_names, values)))
            except (AttributeError, RuntimeError, ValueError, TypeError):
                self.gate.identity({})

    return MapWatch


def capture(args, rclpy):
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from task1_fusion.task3_quality import StableStartWindow
    base = load_config(args.base_config)
    if args.out_dir.exists():
        raise ValueError('output directory already exists; an existing origin will not be replaced')
    # This provisional identity is only used for checks, never saved or published.
    provisional = bind_origin(base, args.map, [0., 0., 0.], [0., 0., 0., 1.],
                              localizer_node=args.localizer_node)
    p, projector = validate(provisional)
    watch = ros_watch_class()(p, provisional['task3_fixed_map'])
    window = StableStartWindow(projector, min_samples=5, max_gap_sec=p['input_timeout_sec'])

    def receive(msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (watch.get_clock().now().nanoseconds-stamp)*1e-9
        if (not watch.ready() or msg.header.frame_id != 'map' or msg.child_frame_id != 'body'
                or stamp <= 0 or not -p['future_tolerance_sec'] <= age <= p['input_timeout_sec']):
            return
        pos, q = msg.pose.pose.position, msg.pose.pose.orientation
        try:
            window.add(stamp, [pos.x, pos.y, pos.z], [q.x, q.y, q.z, q.w])
        except ValueError:
            pass

    watch.create_subscription(Odometry, p['input_topic'], receive, qos_profile_sensor_data)
    try:
        print('仅首次绑定：正确车架角点投影对选定正式O，图示轴对齐场地轴，保持车静止。', flush=True)
        print('等待旧地图定位及连续约2秒低变化样本；不会用当前启动位置自动代替正式O。', flush=True)
        deadline, estimate = time.monotonic()+30., None
        while time.monotonic() < deadline:
            rclpy.spin_once(watch, timeout_sec=.1)
            if not watch.ready():
                window.rows.clear()
                window.last_stamp = None
                continue
            estimate = window.estimate()
            if estimate is not None:
                break
        if estimate is None:
            raise ValueError('no accepted stationary map window: '+watch.gate.reason+'; origin not saved')
        result = bind_origin(base, args.map, estimate['source_xyz_m'],
                             estimate['source_quaternion_xyzw'], localizer_node=args.localizer_node)
        args.out_dir.mkdir(parents=True, exist_ok=False)
        config = args.out_dir/'task3_corner.yaml'
        config.write_text(yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding='utf-8')
        report = {
            'created_at_utc': datetime.now(timezone.utc).isoformat(),
            'data_kind': 'live_map_localizer_stationary_binding',
            'base_config': str(args.base_config.resolve()), 'base_config_sha256': sha256(args.base_config),
            'map_file': p['map_file'], 'map_sha256': p['expected_map_sha256'],
            'reference_id': result['task3_vision']['ros__parameters']['reference_id'],
            'source_estimate': estimate,
            'source_samples': [{'stamp_ns': row[0], 'xyz': row[1].tolist(), 'quaternion_xyzw': row[2].tolist()}
                               for row in window.rows],
            'config_sha256': sha256(config), 'physical_accuracy_verified': False,
            'scope': 'Operator confirms chosen O and field heading; stationary binding is not accuracy acceptance.',
        }
        (args.out_dir/'map_reference.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print('固定原点已保存：'+str(config.resolve()))
        print('以后使用run/ui加载此配置，不再执行bind，不再建立启动点。')
    finally:
        watch.destroy_node()


def run(config, rclpy):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from task1_fusion.corner_node import CornerNode
    from task1_fusion.vision_node import VisionNode
    p, _ = validate(config)
    watch = ros_watch_class()(p, config['task3_fixed_map'])
    # Allow graph discovery before checking for an existing interface.
    deadline = time.monotonic()+2.
    while time.monotonic() < deadline:
        rclpy.spin_once(watch, timeout_sec=.1)
    if any(watch.count_publishers(t) for t in ('/vision/state', p['output_prefix']+'/odom')):
        watch.destroy_node()
        raise ValueError('corner/vision interface already running; stop that interface first')

    class MapCorner(CornerNode):
        def callback(self, msg):
            if not watch.ready():
                self.reject(watch.gate.reason)
                return
            super().callback(msg)

        def diagnostics(self):
            if not watch.ready():
                self.reason = watch.gate.reason
            super().diagnostics()

    nodes, executor = [watch], SingleThreadedExecutor()
    try:
        corner = MapCorner(parameter_overrides=[Parameter(k, value=v) for k, v in p.items()])
        nodes.append(corner)
        v = dict(config.get('task3_vision', {}).get('ros__parameters', {}))
        v.update(input_topic=p['output_prefix']+'/odom', input_parent_frame=p['output_frame'],
                 input_child_frame=p['output_child_frame'], diagnostics_topic='/task3/diagnostics',
                 input_timeout_sec=p['input_timeout_sec'], future_tolerance_sec=p['future_tolerance_sec'])
        vision = VisionNode(parameter_overrides=[Parameter(k, value=value) for k, value in v.items()])
        nodes.append(vision)
        watch.interface_started = True
        for node in nodes:
            executor.add_node(node)
        print('固定地图模式：保留正式O；等待旧地图定位，启动位置不会清零。', flush=True)
        executor.spin()
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='command', required=True)
    bind = subs.add_parser('bind', help='ONE-TIME: chosen corner on formal O and axes aligned')
    bind.add_argument('--map', type=Path, required=True)
    bind.add_argument('--base-config', type=Path, default=ROOT/'config/car_task3_local.yaml')
    bind.add_argument('--localizer-node', default='/map_localizer')
    bind.add_argument('--out-dir', type=Path, required=True)
    bind.add_argument('--on-formal-origin', action='store_true', required=True,
                      help='explicit physical placement/heading acknowledgement; not a daily step')
    for name in ('check', 'run', 'ui'):
        sub = subs.add_parser(name)
        sub.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command != 'bind':
            config = load_config(args.config)
            p, _ = validate(config)
            if args.command == 'check':
                print(json.dumps({'mode': 'fixed_map', 'map_file': p['map_file'],
                    'reference_id': config['task3_vision']['ros__parameters']['reference_id'],
                    'output_frame': p['output_frame'], 'startup_zeroing': False,
                    'scope': 'offline file/geometry check only; no localization or accuracy acceptance'},
                    ensure_ascii=False, indent=2))
                return 0
        import rclpy
        rclpy.init(args=[])
        try:
            if args.command == 'bind':
                capture(args, rclpy)
            elif args.command == 'run':
                run(config, rclpy)
            else:
                import tkinter as tk
                from task1_fusion.vision_ui import VisionWindow
                root = tk.Tk()
                window = VisionWindow(root, str(args.config.resolve()), '', operator_mode=True,
                                      fixed_map_script=str(Path(__file__).resolve()))
                try:
                    root.mainloop()
                finally:
                    window.cleanup()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
