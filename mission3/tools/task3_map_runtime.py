#!/usr/bin/env python3
"""Own a map localizer and an automatic global initializer; never move the car."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/task1_fusion'))
from task1_fusion.task3_config import DEFAULTS, sha256
from task3_runtime import OwnedProcesses, raw_source_ids
from task3_fixed_map import ros_watch_class


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('launch_args', nargs='*')
    args = parser.parse_args()
    import rclpy
    rclpy.init(args=[])
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    lock = open(Path(tempfile.gettempdir())/
                f'task3_map_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID", "0")}.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('自动地图定位入口已运行，请使用原窗口或终端。')
    def stop_signal(*_):
        raise KeyboardInterrupt
    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, stop_signal)
    map_file = args.map.resolve()
    out = ROOT/'results'/('task3_auto_start_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
    processes = OwnedProcesses(ROOT, out/'logs')
    p = {**DEFAULTS, 'map_file': str(map_file), 'expected_map_sha256': sha256(map_file)}
    watch = ros_watch_class()(p, {'localizer_node': '/map_localizer', 'quality_topic': '/map_lidar/quality'},
                             node_name='task3_map_startup_watch')
    initialized, sources, map_stamp = False, None, [0]
    def observe_map(msg):
        if msg.header.frame_id == 'map' and msg.child_frame_id == 'body':
            map_stamp[0] = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    watch.create_subscription(Odometry, '/map_lidar/odom', observe_map, qos_profile_sensor_data)
    status_pub = None
    def status():
        if status_pub is not None and rclpy.ok():
            status_pub.publish(String(data=json.dumps({
                'schema_version': 1, 'stamp_ns': watch.get_clock().now().nanoseconds,
                'map_sha256': p['expected_map_sha256'], 'source_ids': sources or {},
                'initialized': initialized, 'tracking': initialized and watch.ready(),
            })))
    watch.create_timer(.1, status)
    try:
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            rclpy.spin_once(watch, timeout_sec=.1)
        if watch.count_publishers('/task3/map_startup/state'):
            raise ValueError('已有其他自动地图入口运行，拒绝重复启动。')
        sources = raw_source_ids(watch)
        if sources is None:
            raise ValueError('请先运行且仅运行一份FAST-LIO。')
        status_pub = watch.create_publisher(String, '/task3/map_startup/state', 10)
        count = watch.count_publishers('/map_lidar/quality')
        if count > 1:
            raise ValueError('存在多份地图定位程序；请关闭重复的一份。')
        if count == 0:
            processes.start('map_localizer', ['ros2', 'launch', 'lidar_map_localization',
                'map_localizer.launch.py', 'map_path:='+str(map_file),
                'auto_initialize_identity:=false', 'initial_guess_enabled:=false', *args.launch_args])
            print('正在加载旧地图；随后自动搜索当前位置，无需在RViz点位置。', flush=True)
        else:
            print('检测到已有地图定位程序，复用它。', flush=True)
        # A map->camera_init from another boot must not be trusted merely because
        # the new source uses the same frame name. Search once for this session.
        deadline = time.monotonic()+3.
        while not watch.ready() and time.monotonic() < deadline:
            rclpy.spin_once(watch, timeout_sec=.1)
        processes.start('automatic_initialization', [sys.executable,
            str(ROOT/'tools/task3_auto_relocalize.py'), '--map', str(map_file),
            '--out-dir', str(out/'global_search'), '--initialize'])
        print('正在自动定位，请保持车静止。日志：'+str(out/'logs/automatic_initialization.log'), flush=True)
        child = processes.children['automatic_initialization']
        while child.poll() is None:
            rclpy.spin_once(watch, timeout_sec=.1)
            if raw_source_ids(watch) != sources:
                raise ValueError('FAST-LIO会话发生变化；请关闭程序后重新启动自动定位。')
            map_child = processes.children.get('map_localizer')
            if map_child is not None and map_child.poll() is not None:
                raise ValueError('地图定位进程已退出，查看日志：'+str(out/'logs/map_localizer.log'))
        if child.returncode:
            raise ValueError('自动搜索未通过，未定位；查看日志：'+str(out/'logs/automatic_initialization.log'))
        processes.children.pop('automatic_initialization')
        decision = json.loads((out/'global_search/decision.json').read_text())
        init_stamp = decision['initial_source_stamp_ns']
        deadline = time.monotonic()+15.
        while (not watch.ready() or map_stamp[0] < init_stamp) and time.monotonic() < deadline:
            rclpy.spin_once(watch, timeout_sec=.1)
        if not watch.ready() or map_stamp[0] < init_stamp:
            raise ValueError('自动候选已送出，但地图精配准尚未通过；保持坐标不可用，查看日志。')
        initialized = True
        print('旧地图自动定位已就绪。正式原点已绑定时可直接打开固定地图坐标窗口。', flush=True)
        print('本终端保持运行；Ctrl+C只关闭本入口启动的进程。', flush=True)
        while rclpy.ok():
            rclpy.spin_once(watch, timeout_sec=.1)
            if raw_source_ids(watch) != sources:
                initialized = False
                status()
                raise ValueError('FAST-LIO已重启或断开，停止使用旧地图变换；重新启动程序即可，不需回O。')
            error = processes.failed()
            if error:
                raise ValueError(error)
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    finally:
        initialized = False
        status()
        processes.stop()
        watch.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
