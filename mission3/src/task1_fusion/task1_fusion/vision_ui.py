"""Local desktop monitor and launcher; never stops an attached external stack."""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


REASONS = {
    'tracking': '坐标有效', 'waiting_for_input': '等待定位输入',
    'input_timed_out': '定位输入已超时',
    'upstream_status_timed_out': '定位状态未连接或已超时',
    'extrinsics_unconfirmed': '安装参数尚未确认，视觉输出无效',
    'upstream_not_tracking': '上游定位状态异常',
    'tracking_unverified_extrinsics': '坐标可接收；按机械参数计算，动态精度未验收',
}


class VisionWindow:
    def __init__(self, root, corner_config, fusion_config, relative_start_script=None,
                 monitor_only=False, operator_mode=False):
        import tkinter as tk
        from tkinter import ttk, filedialog, messagebox
        self.root, self.tk, self.messagebox = root, tk, messagebox
        self.node = Node('task3_vision_desktop')
        self.node.create_subscription(String, '/vision/state', self.receive, 20)
        self.process = self.log_file = None
        self.log_position = 0
        self.last_receipt = None
        self.stop_time = None
        self.closing = False
        self.node_destroyed = False
        self.relative_start_script = relative_start_script
        self.monitor_only = monitor_only
        self.restart_requested = False
        self.corner_config = tk.StringVar(value=corner_config)
        self.fusion_config = tk.StringVar(value=fusion_config)
        self.input_mode = tk.StringVar(value='lidar' if relative_start_script else 'fused')
        self.start_fusion = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value='未连接视觉接口')
        self.values = tk.StringVar(value='X: — m     Y: — m     yaw: — °')
        self.details = tk.StringVar(value='参考点：约定角点；视觉侧自行转换到车心')
        root.title('任务三 · 视觉坐标接口')
        root.geometry('820x580')
        root.minsize(700, 480)
        panel = ttk.Frame(root, padding=16)
        panel.pack(fill='both', expand=True)
        ttk.Label(panel, text='视觉坐标 · 约定角点', font=('', 18)).pack(anchor='w')
        ttk.Label(panel, textvariable=self.status, font=('', 13)).pack(anchor='w', pady=(10, 4))
        ttk.Label(panel, textvariable=self.values, font=('', 18)).pack(anchor='w', pady=8)
        ttk.Label(panel, textvariable=self.details, wraplength=760).pack(anchor='w', pady=(0, 14))
        form = ttk.Frame(panel)
        if not operator_mode:
            form.pack(fill='x')
        form.columnconfigure(1, weight=1)
        fields = [('机械参数配置' if relative_start_script else '角点配置', self.corner_config)]
        if not relative_start_script and not monitor_only:
            fields.append(('任务一配置', self.fusion_config))
        for row, (label, variable) in enumerate(fields if not monitor_only else []):
            ttk.Label(form, text=label).grid(row=row, column=0, sticky='w', padx=(0, 8), pady=4)
            ttk.Entry(form, textvariable=variable).grid(row=row, column=1, sticky='ew')
            def browse(variable=variable):
                path = filedialog.askopenfilename(filetypes=[('YAML 配置', '*.yaml'), ('所有文件', '*')])
                if path:
                    variable.set(path)
            ttk.Button(form, text='选择', command=browse).grid(row=row, column=2, padx=(8, 0))
        modes = ttk.Frame(panel)
        if not relative_start_script and not monitor_only:
            modes.pack(fill='x', pady=8)
        ttk.Radiobutton(modes, text='任务一融合结果', value='fused', variable=self.input_mode).pack(side='left')
        ttk.Radiobutton(modes, text='雷达定位结果', value='lidar', variable=self.input_mode).pack(side='left', padx=12)
        if not relative_start_script and not monitor_only:
            ttk.Checkbutton(panel, text='同时启动任务一融合（原雷达链路和码盘输入须已运行）',
                            variable=self.start_fusion).pack(anchor='w')
        buttons = ttk.Frame(panel)
        buttons.pack(fill='x', pady=12)
        self.start_button = ttk.Button(buttons,
                                      text='建立起点并启动' if relative_start_script else '启动接口',
                                      command=self.start)
        self.start_button.pack(side='left')
        self.stop_button = ttk.Button(buttons, text='停止本窗口启动的接口', command=self.stop, state='disabled')
        self.stop_button.pack(side='left', padx=8)
        self.reset_button = None
        if relative_start_script:
            self.reset_button = ttk.Button(buttons, text='重新建立起点',
                                           command=self.reset_start, state='disabled')
            self.reset_button.pack(side='left')
            ttk.Label(panel, text='车静止时建立起点：初始角点X/Y=0，初始yaw=0；坐标轴之后固定。').pack(anchor='w')
        if monitor_only:
            self.start_button.configure(state='disabled')
        ttk.Label(panel, text='已有接口时自动显示；窗口不会关闭其他人启动的定位进程。').pack(anchor='w')
        self.log_text = tk.Text(panel, height=8, wrap='word', state='disabled')
        self.log_text.pack(fill='both', expand=True, pady=(8, 0))
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(50, self.tick)

    def receive(self, msg):
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict):
                raise ValueError('vision state must be an object')
            if state.get('schema_version') != 1 or state.get('reference_point') != 'corner':
                raise ValueError('unexpected vision interface')
            self.last_receipt = time.monotonic()
            self.status.set(REASONS.get(state['reason'], state['reason']))
            if state.get('data_source') == 'synthetic_stationary':
                self.status.set('离线演示（合成静止数据） · '+self.status.get())
            data = state.get('data')
            if state.get('valid') and data is not None:
                x, y, yaw = (float(data[k]) for k in ('x', 'y', 'yaw'))
                if not all(math.isfinite(v) for v in (x, y, yaw)):
                    raise ValueError('invalid coordinates')
                self.values.set(f'X: {x:.4f} m     Y: {y:.4f} m     yaw: {math.degrees(yaw):.2f} °')
                self.details.set(f'参考点：约定角点    坐标系：{data["frame_id"]}    '
                                 f'数据年龄：{state["source_age_sec"]:.3f} s\n'
                                 f'起点编号：{state.get("reference_id", "未提供")}    Z不作为有效高度')
            else:
                self.values.set('X: — m     Y: — m     yaw: — °')
                self.details.set('参考点：约定角点；当前没有可供视觉使用的有效坐标')
        except (ValueError, KeyError, TypeError):
            self.status.set('收到的视觉接口数据格式异常')
            self.values.set('X: — m     Y: — m     yaw: — °')

    def append_log(self, text):
        self.log_text.configure(state='normal')
        self.log_text.insert('end', text)
        if int(self.log_text.index('end-1c').split('.')[0]) > 600:
            self.log_text.delete('1.0', '200.0')
        self.log_text.see('end')
        self.log_text.configure(state='disabled')

    def start(self):
        if self.monitor_only:
            return
        if self.process is not None:
            return
        if self.node.count_publishers('/vision/state'):
            self.messagebox.showinfo('接口已运行', '已连接现有视觉接口，无需重复启动。')
            return
        corner = Path(self.corner_config.get()).expanduser().resolve()
        fusion = Path(self.fusion_config.get()).expanduser().resolve()
        if not corner.is_file() or (not self.relative_start_script and not fusion.is_file()):
            self.messagebox.showerror('配置不存在', '请选择已存在的角点配置和任务一配置文件。')
            return
        if self.start_fusion.get() and self.input_mode.get() != 'fused':
            self.messagebox.showerror('输入选择', '同时启动任务一时，请选择“任务一融合结果”。')
            return
        if self.relative_start_script:
            command = ['bash', self.relative_start_script, '--base-config', str(corner)]
        elif self.start_fusion.get():
            command = ['ros2', 'launch', 'task1_fusion', 'task1.launch.py',
                       'start_task3:=true', 'start_can:=false', 'record:=false',
                       f'config:={fusion}', f'task3_config:={corner}']
        else:
            command = ['ros2', 'launch', 'task1_fusion', 'task3.launch.py',
                       f'input_mode:={self.input_mode.get()}', f'corner_config:={corner}',
                       f'fusion_config:={fusion}']
        try:
            self.log_file = tempfile.TemporaryFile(mode='w+b')
            self.log_position = 0
            self.process = subprocess.Popen(command, stdout=self.log_file, stderr=subprocess.STDOUT,
                                            stdin=subprocess.DEVNULL, start_new_session=True)
        except OSError as exc:
            if self.log_file is not None:
                self.log_file.close()
                self.log_file = None
            self.messagebox.showerror('启动失败', str(exc))
            return
        self.start_button.configure(state='disabled')
        self.stop_button.configure(state='normal')
        if self.reset_button is not None:
            self.reset_button.configure(state='normal')
        self.status.set('接口启动中，等待定位数据')

    def stop(self):
        if self.process is not None and self.process.poll() is None and self.stop_time is None:
            # No terminal stdin: launch forwards this SIGINT to its own nodes,
            # rather than assuming a terminal already signalled the whole group.
            try:
                self.process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            self.stop_time = time.monotonic()

    def reset_start(self):
        if self.process is not None:
            self.restart_requested = True
            self.stop()

    def close(self):
        self.closing = True
        self.restart_requested = False
        self.stop()

    def tick(self):
        try:
            for _ in range(10):
                rclpy.spin_once(self.node, timeout_sec=0.)
            if self.last_receipt is None or time.monotonic()-self.last_receipt > .7:
                self.status.set('未连接视觉接口，或接口心跳已超时')
                self.values.set('X: — m     Y: — m     yaw: — °')
            if self.log_file is not None:
                # pread does not change the descriptor offset used by child stdout.
                import os
                data = os.pread(self.log_file.fileno(), 8192, self.log_position)
                self.log_position += len(data)
                if data:
                    self.append_log(data.decode('utf-8', errors='replace'))
            if self.process is not None:
                if self.stop_time is not None and time.monotonic()-self.stop_time > 6:
                    import os
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                result = self.process.poll()
                if result is not None:
                    self.append_log(f'\n接口进程结束，退出码 {result}\n')
                    self.process, self.stop_time = None, None
                    self.log_file.close()
                    self.log_file = None
                    self.start_button.configure(state='normal')
                    self.stop_button.configure(state='disabled')
                    if self.reset_button is not None:
                        self.reset_button.configure(state='disabled')
                    if self.restart_requested and not self.closing:
                        self.restart_requested = False
                        self.last_receipt = None
                        self.values.set('X: — m     Y: — m     yaw: — °')
                        self.start()
            if self.closing and self.process is None:
                self.node.destroy_node()
                self.node_destroyed = True
                self.root.destroy()
                return
            self.root.after(50, self.tick)
        except (rclpy.executors.ExternalShutdownException, KeyboardInterrupt):
            self.close()
            self.root.after(50, self.tick)

    def cleanup(self):
        if self.process is not None:
            self.stop()
            try:
                self.process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                import os
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.process.wait(timeout=2)
            self.process = None
        if self.log_file is not None:
            self.log_file.close()
        if not self.node_destroyed:
            self.node.destroy_node()
            self.node_destroyed = True


def main():
    share = Path(get_package_share_directory('task1_fusion'))
    parser = argparse.ArgumentParser(description='任务三视觉坐标桌面窗口')
    parser.add_argument('--corner-config', default=str(share/'config/task3_corner.yaml'))
    parser.add_argument('--fusion-config', default=str(share/'config/fusion.yaml'))
    parser.add_argument('--relative-start-script', help='current car start-frame entry script')
    parser.add_argument('--monitor-only', action='store_true')
    args = parser.parse_args()
    try:
        import tkinter as tk
        root = tk.Tk()
    except (ImportError, RuntimeError) as exc:
        raise SystemExit(f'无法打开桌面窗口；需要图形桌面和 python3-tk：{exc}')
    except tk.TclError as exc:
        raise SystemExit(f'无法连接图形桌面；请在小车本机桌面运行：{exc}')
    rclpy.init(args=[])
    window = None
    try:
        window = VisionWindow(root, args.corner_config, args.fusion_config,
                              args.relative_start_script, args.monitor_only)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if window is not None:
            window.cleanup()
        if rclpy.ok():
            rclpy.shutdown()
