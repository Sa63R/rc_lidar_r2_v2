#!/usr/bin/env python3
"""One operator window owns startup; it never silently creates a field origin."""
from datetime import datetime
import fcntl
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/task1_fusion'))
from task3_runtime import OwnedProcesses, Startup


def main():
    import tkinter as tk
    from tkinter import ttk, messagebox
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from task1_fusion.vision_ui import VisionWindow

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        raise RuntimeError('无法连接图形桌面：'+str(exc)) from exc
    root.withdraw()
    lock = open(Path(tempfile.gettempdir())/
                f'task3_operator_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID", "0")}.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        messagebox.showinfo('已经启动', '任务三一键窗口已经运行，请使用已有窗口。', parent=root)
        root.destroy()
        lock.close()
        return 2
    log_dir = ROOT/'.logs'/('operator_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
    processes = OwnedProcesses(ROOT, log_dir)
    startup = Startup(processes)
    rclpy.init(args=[])

    class OperatorWindow(VisionWindow):
        def __init__(self):
            bar = ttk.Frame(root, padding=12)
            bar.pack(fill='x')
            self.connection = tk.StringVar(value=startup.message)
            ttk.Label(bar, textvariable=self.connection, wraplength=780).pack(anchor='w')
            ttk.Button(bar, text='使用说明', command=lambda: subprocess.Popen(
                ['xdg-open', str(ROOT/'docs/00_使用说明.html')])).pack(side='left', pady=6)
            ttk.Button(bar, text='查看运行日志', command=lambda: subprocess.Popen(
                ['xdg-open', str(log_dir)])).pack(side='left', padx=8)
            ttk.Button(bar, text='停止整套程序并退出', command=self.close).pack(side='right')
            super().__init__(root, str(ROOT/'config/car_task3_local.yaml'),
                             str(ROOT/'src/task1_fusion/config/fusion.yaml'),
                             str(ROOT/'scripts/start_car_task3_relative.sh'), operator_mode=True)
            root.title('任务三 · 视觉组日常使用')
            root.geometry('880x700')
            self.start_button.configure(state='disabled')
            for kind, topic, msg_type in [('lidar', '/livox/lidar', CustomMsg),
                                          ('imu', '/livox/imu', Imu),
                                          ('odom', '/Odometry', Odometry)]:
                self.node.create_subscription(msg_type, topic,
                    lambda msg, kind=kind: self.observe(kind, msg), qos_profile_sensor_data)
            root.after(100, self.connection_tick)

        def observe(self, kind, msg):
            pose = msg.pose.pose if kind == 'odom' else None
            values = ([pose.position.x, pose.position.y, pose.position.z,
                       pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                      if pose is not None else [])
            finite = all(math.isfinite(v) for v in values)
            if pose is not None:
                norm = sum(v*v for v in values[3:])
                finite = finite and .99 < norm < 1.01
            startup.observe(kind, msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                            self.node.get_clock().now().nanoseconds,
                            frame=msg.header.frame_id,
                            child=msg.child_frame_id if kind == 'odom' else '', pose_finite=finite)

        def connection_tick(self):
            if self.closing:
                return
            counts = {k: self.node.count_publishers(t) for k, t in [
                ('lidar', '/livox/lidar'), ('imu', '/livox/imu'),
                ('odom', '/Odometry'), ('vision', '/vision/state')]}
            try:
                was_ready = startup.ready
                startup.step(counts)
            except OSError as exc:
                startup.fail('无法启动程序：'+str(exc))
                was_ready = False
            if was_ready and not startup.ready and self.process is not None:
                # A restarted source may have a different local world frame.
                # Stop this window's interface instead of reusing its old origin.
                self.restart_requested = False
                self.stop()
                self.append_log('输入中断，已停止本窗口的坐标接口。恢复后回到正式O并摆正朝向，重新建立起点。\n')
            self.connection.set(startup.message)
            if self.process is None:
                self.start_button.configure(state='normal' if startup.ready else 'disabled')
            if self.reset_button is not None:
                self.reset_button.configure(state='normal' if startup.ready and self.process else 'disabled')
            root.after(200, self.connection_tick)

        def start(self):
            if not startup.ready:
                messagebox.showinfo('定位尚未就绪', startup.message, parent=root)
                return
            super().start()

        def reset_start(self):
            if messagebox.askyesno('确认更换原点',
                    '这会改变视觉使用的坐标基准。已将正确角点对准正式原点，并把初始朝向摆正了吗？', parent=root):
                super().reset_start()

    window = None
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        window = OperatorWindow()
        signal.signal(signal.SIGINT, lambda *_: root.after(0, window.close))
        signal.signal(signal.SIGTERM, lambda *_: root.after(0, window.close))
        signal.signal(signal.SIGHUP, lambda *_: root.after(0, window.close))
        root.deiconify()
        root.mainloop()
        return 2 if startup.stage == 'failed' else 0
    finally:
        if window is not None:
            window.cleanup()
        processes.stop()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ImportError, RuntimeError) as exc:
        raise SystemExit(f'无法启动任务三窗口：{exc}。请在已部署主机的图形桌面运行。')
