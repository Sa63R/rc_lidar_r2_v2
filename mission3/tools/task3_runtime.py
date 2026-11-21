"""Process ownership and fresh-input startup for the task-3 operator window."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time


class OwnedProcesses:
    def __init__(self, root, log_dir, spawn=subprocess.Popen):
        self.root, self.log_dir, self.spawn = Path(root), Path(log_dir), spawn
        self.children = {}

    def start(self, name, command=None):
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stream = (self.log_dir/(name+'.log')).open('ab')
        try:
            child = self.spawn(command or ['bash', str(self.root/'bin/task3'), name],
                               stdin=subprocess.DEVNULL, stdout=stream,
                               stderr=subprocess.STDOUT, start_new_session=True)
        finally:
            stream.close()
        self.children[name] = child

    def failed(self):
        for name, child in self.children.items():
            result = child.poll()
            if result is not None:
                return f'{name}已退出（代码{result}），请查看日志。'
        return ''

    def stop(self, grace=(5, 2, 1)):
        # Only process groups created by this instance are ever signalled.
        for child in reversed(list(self.children.values())):
            for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), grace):
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    child.wait(timeout=timeout)
                    # The parent launcher can exit before its children.
                    # Wait for the owned group, then escalate if needed.
                    deadline = time.monotonic()+timeout
                    while time.monotonic() < deadline:
                        try:
                            os.killpg(child.pid, 0)
                        except ProcessLookupError:
                            break
                        time.sleep(.02)
                    else:
                        continue
                    break
                except subprocess.TimeoutExpired:
                    continue
        self.children.clear()


class Startup:
    """No ROS imports: the UI supplies graph counts and timestamped observations."""
    def __init__(self, processes, now=time.monotonic):
        self.processes, self.now = processes, now
        self.stage, self.deadline = 'discover', now()+2.5
        self.samples = {}
        self.ready = False
        self.message = '正在检查雷达和定位程序，请保持车静止。'

    def observe(self, kind, stamp_ns, ros_now_ns, *, frame='', child='', pose_finite=True):
        age = (ros_now_ns-stamp_ns)*1e-9
        valid = math.isfinite(age) and -.05 <= age <= .5
        if kind == 'odom':
            valid = valid and frame == 'camera_init' and child == 'body' and pose_finite
        if valid:
            self.samples[kind] = self.now()+.5-max(age, 0.)
        else:
            self.samples.pop(kind, None)

    def fresh(self, kind):
        return kind in self.samples and self.now() <= self.samples[kind]

    def fail(self, message):
        self.stage, self.ready, self.message = 'failed', False, message

    def step(self, counts):
        if self.stage == 'failed':
            return
        failure = self.processes.failed()
        if failure:
            self.fail(failure)
            return
        if any(counts.get(k, 0) > 1 for k in ('lidar', 'imu', 'odom', 'vision')):
            self.fail('发现重复的数据发布程序。请关闭多开的一份，再重新打开本窗口。')
            return
        if self.stage == 'discover':
            if self.now() < self.deadline:
                return
            if not counts.get('lidar') and not counts.get('imu'):
                self.processes.start('lidar')
            self.stage, self.deadline = 'lidar', self.now()+30
            self.message = '正在等待雷达点云和IMU；请保持供电、网线连接，车保持静止。'
        elif self.stage == 'lidar':
            if self.fresh('lidar') and self.fresh('imu'):
                if not counts.get('odom'):
                    self.processes.start('fastlio')
                self.stage, self.deadline = 'odom', self.now()+45
                self.message = '正在初始化定位，请保持车静止。'
            elif self.now() > self.deadline:
                self.fail('没有收到新鲜雷达数据。检查供电、网线和主机有线IP后，关闭并重新打开本窗口。')
        elif self.stage == 'odom':
            if all(self.fresh(k) for k in ('lidar', 'imu', 'odom')):
                self.stage = 'ready'
            elif self.now() > self.deadline:
                self.fail('定位未就绪。确认车静止；查看日志后关闭并重新启动，勿建立起点。')
        if self.stage == 'ready':
            self.ready = (all(self.fresh(k) for k in ('lidar', 'imu', 'odom'))
                          and all(counts.get(k) == 1 for k in ('lidar', 'imu', 'odom')))
            self.message = ('定位已就绪：摆正角点和初始朝向，点击“建立起点并启动”。'
                            if self.ready else '输入暂时中断，暂停使用坐标；等待恢复或查看日志。')


def raw_source_ids(node):
    """Identify the actual LIO publishers, not just their reusable frame names."""
    result = {}
    for topic in ('/Odometry', '/cloud_registered'):
        pubs = node.get_publishers_info_by_topic(topic)
        if len(pubs) != 1:
            return None
        result[topic] = list(pubs[0].endpoint_gid)
    return result


class MapSession:
    """Fresh startup status for THIS map and THIS LIO session (no ROS imports)."""
    def __init__(self, expected_map_sha256, now=time.monotonic):
        self.expected_map_sha256, self.now = expected_map_sha256, now
        self.until = -math.inf
        self.sources = None
        self.initialized = self.tracking = False

    def observe(self, state, ros_now_ns):
        try:
            age = (ros_now_ns-int(state['stamp_ns']))*1e-9
            valid = (state['schema_version'] == 1 and state['map_sha256'] == self.expected_map_sha256
                     and -.05 <= age <= .5 and isinstance(state['source_ids'], dict))
        except (KeyError, ValueError, TypeError):
            valid = False
        if not valid:
            self.until, self.initialized, self.tracking = -math.inf, False, False
            return
        self.until = self.now()+.5-max(age, 0.)
        self.sources = state['source_ids']
        self.initialized = state.get('initialized') is True
        self.tracking = state.get('tracking') is True

    def live(self, sources):
        return (sources is not None and self.now() <= self.until
                and self.sources == sources and self.initialized)

    def ready(self, sources):
        return self.live(sources) and self.tracking
