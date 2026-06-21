# R2 全链路迁移执行剧本

本文档给另一个 Codex/agent 在 r2 机器上执行，用来复刻当前 Orange Pi 上已经跑通的 RC LIDAR 流程。目标不是复制实验日志，而是把环境、启动策略、建图、重定位、CAN、滤波、健康日志和排查方法按 r2 实机状态重新落地。

## 迁移原则

- 以 r2 实机检查结果为准。当前仓库只能作为参考模板，不能直接照搬网口名、IP、CAN 接口、地图路径、机械外参和开机自启路径。
- 不迁移实验报告、历史日志、临时 PlotJuggler 布局状态、`note.md`、`logs/`、`report_assets/`、`maps/*.pcd`，除非用户明确要求拷贝某一份地图。
- 当前对外业务坐标系为 `X右/Y前/Z上`，yaw 为左转正。FAST-LIO/Livox 原始帧仍保持算法坐标，只通过静态 TF 生成机器人业务帧。
- r2 默认生产启动策略是低负载重定位：不启动 RViz，不启动 PlotJuggler，CAN 发送 `/robot/pose_filtered`。
- 所有会改系统网络、udev、sudoers 或开机自启的动作，执行前必须先读取 r2 的当前配置；确认是 r2 目标机器后再执行。
- 后续维护规则：凡是当前仓库更新了 `readme.md` 里的运行方式、默认环境变量、启动链路、CAN 输出、PlotJuggler、滤波、健康日志、建图/重定位或排查命令，也要同步更新本文档里的 r2 迁移步骤，让 r2 能按同样思路复刻。

## 1. 定位 r2 工作区和当前状态

在 r2 上先确定目标仓库根目录，并用变量贯穿后续命令：

```bash
if git rev-parse --show-toplevel >/dev/null 2>&1; then
  export R2_WS="$(git rev-parse --show-toplevel)"
elif [ -d "$HOME/RC_LIDAR_R2/.git" ]; then
  export R2_WS="$HOME/RC_LIDAR_R2"
elif [ -d "$HOME/rc_lidar_r2/.git" ]; then
  export R2_WS="$HOME/rc_lidar_r2"
else
  echo "ERROR: cannot find r2 workspace. Clone or cd into the r2 repository first."
  exit 2
fi

cd "$R2_WS"
pwd
git status -sb
git remote -v
```

如果工作区有未提交改动，先不要覆盖。记录差异：

```bash
git diff --stat
git status --short
```

确认系统和 ROS：

```bash
uname -a
lsb_release -a 2>/dev/null || cat /etc/os-release
dpkg --print-architecture
test -f /opt/ros/humble/setup.bash && echo "ROS Humble found"
source /opt/ros/humble/setup.bash
ros2 --version
```

## 2. 安装和恢复基础环境

安装基础依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-rosidl-default-generators \
  ros-humble-rclcpp-components \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-robot-localization \
  ros-humble-plotjuggler ros-humble-plotjuggler-ros xdotool \
  libpcl-dev libeigen3-dev libboost-all-dev libapr1-dev \
  can-utils usbutils ethtool arping
```

初始化 `rosdep`，如果已经初始化则跳过报错：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
cd "$R2_WS"
rosdep install --from-paths src --ignore-src -r -y
```

检查 Livox-SDK2：

```bash
test -f /usr/local/include/livox_lidar_api.h
test -f /usr/local/lib/liblivox_lidar_sdk_shared.so
```

如果缺失，安装 SDK：

```bash
cd "$HOME"
test -d Livox-SDK2 || git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
sudo make install
```

编译工作区：

```bash
cd "$R2_WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DROS_EDITION=ROS2 \
  -DHUMBLE_ROS=humble
source install/setup.bash
```

验证 7 个核心包：

```bash
for p in fast_lio livox_ros_driver2 lidar_map_localization lidar_localization rc_serial_driver fastlio_world_init my_static_tf; do
  echo "== $p =="
  ros2 pkg executables "$p"
done
```

## 3. 适配 r2 雷达网络

先查 r2 网口，不要假设一定是 `eth0`：

```bash
ip -br link
ip -br addr
```

找到接 MID360 的网口，记为 `$R2_LIVOX_IFACE`。如果不确定，拔插雷达网线对比 `ip -br link` 和 `ethtool`：

```bash
export R2_LIVOX_IFACE="eth0"
ip -br link show "$R2_LIVOX_IFACE"
ethtool "$R2_LIVOX_IFACE" | grep -i "Link detected"
```

当前模板默认：

```text
主机 IP: 192.168.1.5
雷达 IP: 192.168.1.124
```

在 r2 上临时配置并验证：

```bash
sudo ip link set "$R2_LIVOX_IFACE" up
sudo ip addr add 192.168.1.5/24 dev "$R2_LIVOX_IFACE" 2>/dev/null || true
ip route get 192.168.1.124
arping -I "$R2_LIVOX_IFACE" -c 3 192.168.1.124
```

如果 `arping` 无回应，优先检查雷达供电、网线、交换机/转接盒、网口是否选错。不要先改 TF。

检查配置文件里的 host IP 和 lidar IP：

```bash
cd "$R2_WS"
grep -n "192.168" src/livox_ros_driver2/config/MID360_config.json
```

如果 r2 雷达 IP 或主机 IP 不同，修改 `MID360_config.json`，然后重新编译或重新 source。修改原则：

- `host_net_info` 使用 r2 接雷达网口上的本机 IP。
- `lidar_configs` 使用实际 MID360 IP。
- 不要把同一个 `192.168.1.5/24` 配到多个网口。

如果 r2 也使用 NetworkManager 和免密一键启动，执行一次：

```bash
cd "$R2_WS"
./autostart/install_one_click_network.sh
```

执行后再次验证：

```bash
ip -br addr
arping -I "$R2_LIVOX_IFACE" -c 3 192.168.1.124
```

## 4. 适配 r2 CAN 链路

插上 USB-CAN 和电控接收端，检测接口：

```bash
cd "$R2_WS"
./autostart/can_setup.sh --check-only
lsusb
ip -details link show type can
ip -brief link
```

如果检测到 `can0`、`can1` 或 `slcan0`，配置 1Mbps：

```bash
./autostart/can_setup.sh
ip -details -statistics link show type can
```

成功标准：

- 存在 `can*` 或 `slcan*`。
- flags 里有 `UP`。
- 有接收端时，不应长期 bus-off。
- 运行 CAN 发送后，`TX packets` 增加。

如果没有 CAN 接口：

```bash
dmesg --ctime | grep -Ei 'can|peak|pcan|xcan|0c72|usb.*disconnect|error -71|error -62' | tail -n 80
```

处理顺序：

1. 重插 USB-CAN。
2. 换 USB 口。
3. 使用带供电 HUB。
4. 在另一台 Ubuntu 上运行 `can_setup.sh --check-only` 交叉验证适配器。
5. 确认接收端供电、CANH/CANL、终端电阻和波特率。

当前 CAN 报文语义：

```text
0x12: float x_can, float y_can
0x13: float z_can, float yaw_can
x_can = 右方
y_can = 前方
z_can = 上方
yaw_can = 左转为正
```

重定位模式默认：

```text
RC_CAN_POSE_SOURCE=pose_topic
RC_CAN_POSE_TOPIC=/robot/pose_filtered
```

临时回退旧 TF 链路：

```bash
RC_CAN_POSE_SOURCE=tf ./autostart/start_all_terminals.sh
```

## 5. 迁移启动策略和开机自启

安装自启入口时，优先安装 user systemd，自带 XDG 桌面自启只作为备用。两个入口都调用 `autostart/boot_start.sh`，脚本会用 `boot_id` 防止同一次开机重复启动。

```bash
cd "$R2_WS"
./autostart/install_desktop_autostart.sh
./autostart/install_user_systemd_autostart.sh
systemctl --user status rc-lidar-autostart.service
```

R2 上不要直接复制 Orange Pi 的 systemd 用户目录；只运行仓库里的安装脚本。若 R2 没有自动登录图形桌面，user systemd 仍可能需要 R2 的登录/linger 策略配合，先用下面命令确认自启入口有没有被调用：

```bash
tail -n 80 logs/autostart/boot_attempts.log
tail -n 120 logs/autostart/xdg_autostart.log
cat logs/autostart/boot_start_state.env
find logs/autostart -maxdepth 1 -type d -name 'run_*' -printf '%TY-%Tm-%Td %TH:%TM:%TS %f\n' | sort -r | head
```

注意：开机后系统时间可能先不准，之后被 NTP/chrony 校正。排查日志先后时，不要只看 `run_YYYYMMDD_HHMMSS` 文件名；同时看 `start_summary.txt` 里的 `boot_id`、`uptime_sec` 和 `epoch_sec`。

当前 Orange Pi 工程还有一个重定位 ready 提示节点 `READY_NOTIFY`。迁移到 R2 时也建议保留：它订阅 `/robot/pose_filtered` 和 `/map_lidar/quality`，当重定位连续 accepted/converged 且 `fitness` 达标后写：

```text
logs/autostart/<run>/relocalization_ready.json
```

如果 R2 有图形桌面和终端，它还会弹出“RC LiDAR 重定位完成，现在可以移动小车”的小窗口。这个节点不参与定位计算，ready 后自动退出，常态开销近似为零。R2 上需要按现场情况复核这些阈值：

```bash
RC_READY_NOTIFY_STABLE_SEC=3.0
RC_READY_NOTIFY_TIMEOUT_SEC=120.0
RC_READY_NOTIFY_MAX_FITNESS=0.05
RC_READY_NOTIFY=0          # 如 R2 不需要提示窗口，可关闭
```

先做 dry-run：

```bash
cd "$R2_WS"
RC_AUTOSTART_DRY_RUN=1 ./autostart/start_all_terminals.sh
```

生产推荐低负载重定位启动：

```bash
cd "$R2_WS"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
./autostart/start_all_terminals.sh
```

这不会降低雷达频率，只是不拉起 RViz、PlotJuggler 和额外调试窗口。核心链路包括：

```text
TF_STATIC1/2/3
LIVOX
FASTLIO
HIGHRATE_BODY
world_tf
MAP_LOCALIZER
POSE_INTERFACE
POSE_FILTER
HEALTH_MONITOR
CAN_SERIAL
```

停止所有残留进程：

```bash
cd "$R2_WS"
./autostart/stop_all.sh
```

不要只靠关闭终端窗口停止程序。headless、开机自启和 CAN 重启会把进程放到后台。

设置桌面开机自启：

```bash
cd "$R2_WS"
./autostart/install_desktop_autostart.sh
```

检查自启文件：

```bash
ls -l "$HOME/.config/autostart"
grep -R "RC_LIDAR" "$HOME/.config/autostart" -n
```

开机自启默认应满足：

```text
RC_LIDAR_MODE=relocalization
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1
RC_AUTOSTART_HEADLESS=1
RC_AUTOSTART_PLOTJUGGLER=0
RC_CAN_POSE_SOURCE=pose_topic
RC_CAN_POSE_TOPIC=/robot/pose_filtered
RC_HEALTH_MONITOR=1
```

CAN 重启脚本也应保持同样默认策略：

```bash
grep -n "RC_LIDAR_MODE\\|RC_CAN_POSE_TOPIC\\|RC_HEALTH_MONITOR" autostart/restart_lidar_stack.sh
```

## 6. 建图、保存地图和地图质量

建图启动：

```bash
cd "$R2_WS"
RC_LIDAR_MODE=mapping ./autostart/start_all_terminals.sh
```

移动机器人覆盖需要重定位的场地。建图完成后保存：

```bash
./autostart/save_fast_lio_map.sh
```

默认地图：

```text
maps/mid360_map.pcd
```

如果目标路径已经存在，脚本和 FAST-LIO 保存服务会自动生成带时间戳的新文件名，不覆盖旧地图。确认实际保存路径：

```bash
cat logs/autostart/latest/fastlio_map_save_path.txt 2>/dev/null || true
ls -lh maps/*.pcd
```

检查地图质量：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lidar_map_localization pcd_quality_report --map maps/mid360_map.pcd
```

判断方向：

- 点数太少，不适合高精度重定位。
- XYZ 范围明显不符合场地，说明地图保存错误或轨迹漂移。
- Z 结构太弱、地图过于稀疏、动态物体太多，会增加重定位抖动。
- 多次建图后保留评分更好、现场重复性更好的 PCD。

## 7. 重定位验证

确认地图存在：

```bash
ls -lh maps/mid360_map.pcd
```

如果机器人放在建图开始原点附近：

```bash
cd "$R2_WS"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
./autostart/start_all_terminals.sh
```

如果机器人不是从建图原点启动，给粗略初值。这个初值是 `map -> camera_init` 的 FAST-LIO 原始坐标，不是业务坐标：

```bash
cd "$R2_WS"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_INITIAL_GUESS=1 \
RC_RELOCALIZATION_INITIAL_X=0.0 \
RC_RELOCALIZATION_INITIAL_Y=0.0 \
RC_RELOCALIZATION_INITIAL_Z=0.0 \
RC_RELOCALIZATION_INITIAL_YAW=0.0 \
./autostart/start_all_terminals.sh
```

验证 topic：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /map_lidar/quality --once
ros2 topic echo /robot/pose --once
ros2 topic echo /robot/pose_filtered --once
ros2 topic echo /lidar/pose_filter/status --once
```

检查日志：

```bash
tail -n 120 logs/autostart/latest/MAP_LOCALIZER.log
tail -n 120 logs/autostart/latest/POSE_INTERFACE.log
tail -n 120 logs/autostart/latest/POSE_FILTER.log
tail -n 120 logs/autostart/latest/CAN_SERIAL.log
```

成功标准：

- `MAP_LOCALIZER.log` 没有持续 rejected。
- `/map_lidar/quality` 的 `accepted` 为 1 的比例稳定。
- `/robot/pose` 有连续输出。
- `/robot/pose_filtered` 跟随 `/robot/pose`，静止时更稳，运动时不长时间卡死。
- `CAN_SERIAL.log` 显示 `pose_source=pose_topic pose_topic=/robot/pose_filtered`。

## 8. TF、外参和自转校准

当前静态 TF 结构：

```text
world -> init
body -> base_link
body_highrate -> base_link_highrate
```

当前模板外参：

```text
x = -0.240
y =  0.287302
z = -0.760
roll  = 0
pitch = 0
yaw   = pi/2
```

r2 必须按机械实际重新核对雷达相对底盘中心的位置。不要因为当前模板在 Orange Pi 上能跑，就默认 r2 外参完全一样。

检查三处 TF 文件是否一致：

```bash
grep -R "ROBOT_AXES_YAW_RAD\\|--x\\|--y\\|--z\\|--frame-id\\|--child-frame-id" src/my_static_tf/launch -n
```

启动后验证：

```bash
ros2 run tf2_ros tf2_echo world init
ros2 run tf2_ros tf2_echo body base_link
ros2 run tf2_ros tf2_echo body_highrate base_link_highrate
ros2 run tf2_ros tf2_echo init base_link_highrate
```

如果原地旋转时机器人中心在 XY 上画小圆，运行自转校准：

```bash
cd "$R2_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run fastlio_world_init rotation_extrinsic_calibrator \
  --duration 40 \
  --rate 50
```

操作要求：

- 先启动低负载定位栈。
- 等 `world_tf` 初始化完成。
- 让机器人尽量原地慢速自转，至少一圈，推荐 1.5 到 2 圈。
- 脚本结束后读取 `logs/calibration/rotation_extrinsic_<时间>/summary.json` 和 `report.md`。
- 只按可信报告修改 XY 外参；`z`、roll、pitch 和业务 yaw 不通过这次平面自转校准修改。

## 9. PlotJuggler 调试和滤波观察

生产默认不启动 PlotJuggler。需要调试时：

```bash
cd "$R2_WS"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=0 \
RC_AUTOSTART_PLOTJUGGLER=1 \
./autostart/start_all_terminals.sh
```

当前 PlotJuggler 自动启动策略不是“强行画好所有曲线”，而是做三件低风险自动化：

```text
1. 等待关键 ROS topic 出现。
2. 写入 PlotJuggler 用户配置里的 DataStreamROS2/default_topics。
3. 用 xdotool 自动确认 Warning、Start Streaming?、Topic Selector 等弹窗。
```

r2 上需要安装：

```bash
sudo apt install -y ros-humble-plotjuggler ros-humble-plotjuggler-ros xdotool
```

脚本会更新这个文件：

```text
~/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini

[DataStreamROS2]
default_topics=...
```

重定位模式下应默认预选这些 topic：

```text
/robot/pose
/robot/pose_filtered
/map_lidar/quality
/lidar/pose_filter/status
/tf_compare/robot_init_to_base_link
/tf_compare/robot_init_to_base_link_highrate
/tf_compare/lidar_world_to_body
```

如果 Topic Selector 只选中了旧的 body/body_highrate，或者没有看到 `/robot/pose_filtered`，先只刷新 PlotJuggler 配置，不启动 GUI：

```bash
cd "$R2_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

RC_LIDAR_MODE=relocalization \
PLOTJUGGLER_CONFIG_ONLY=1 \
./autostart/plotjuggler_tf_compare.sh

grep -n "default_topics" "$HOME/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini"
```

如果 PlotJuggler 仍然要求手点确认，检查：

```bash
command -v xdotool
echo "$DISPLAY"
tail -n 120 logs/autostart/latest/PLOTJUGGLER.log
```

如果 `xdotool` 不存在，PlotJuggler 仍会启动，但需要手动点击确认窗口。PlotJuggler ROS2 插件没有可靠的完全静默订阅参数，本工程采用“启动前写入 `default_topics` + 自动确认弹窗”的方案。

默认不强制加载右侧曲线分屏布局。原因是自动分屏布局在现场不一定符合调试需求，用户只需要 topic 自动选中即可。如果确实要恢复旧的自动分屏布局：

```bash
cd "$R2_WS"
PLOTJUGGLER_TF_LAYOUT="$R2_WS/src/fastlio_world_init/config/tf_compare_plotjuggler.xml" \
RC_LIDAR_MODE=relocalization \
RC_AUTOSTART_PLOTJUGGLER=1 \
./autostart/start_all_terminals.sh
```

TF 对比发布节点默认只在 PlotJuggler 会启动时自动启动。低负载实车调车时保持默认跳过；如果想在 headless 下只发布对比 topic、不打开 PlotJuggler：

```bash
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
RC_AUTOSTART_TF_COMPARE=1 \
./autostart/start_all_terminals.sh
```

重点观察：

```text
/robot/pose/pose/position/x
/robot/pose_filtered/pose/position/x
/robot/pose/pose/position/y
/robot/pose_filtered/pose/position/y
/map_lidar/quality
/lidar/pose_filter/status
```

滤波原则：

- 静止时应降低小抖动。
- 绕点转圈或连续运动时，应保留连续曲线，不应出现长期平台或锯齿。
- 单帧硬离群点可以被短暂拒绝。
- 连续拒绝后应恢复，不能永久卡死。

需要关闭滤波观察旧行为：

```bash
RC_POSE_FILTER=0 ./autostart/start_all_terminals.sh
```

需要让 CAN 临时绕过滤波：

```bash
RC_CAN_POSE_SOURCE=tf ./autostart/start_all_terminals.sh
```

## 10. 健康日志和负载判断

每次一键启动默认写：

```text
logs/autostart/latest/health_monitor.csv
logs/autostart/latest/health_monitor_summary.json
logs/autostart/latest/health_monitor_run.json
logs/autostart/latest/health_monitor_stop.json
logs/autostart/latest/health_events.jsonl
```

确认：

```bash
ls -lh logs/autostart/latest/health_monitor.*
ls -lh logs/autostart/latest/health_events.jsonl
tail -n 5 logs/autostart/latest/health_monitor.csv
cat logs/autostart/latest/health_monitor_summary.json
cat logs/autostart/latest/health_monitor_run.json
test -f logs/autostart/latest/health_monitor_stop.json && cat logs/autostart/latest/health_monitor_stop.json || echo "no graceful stop marker yet"
```

重点字段：

```text
max_temp_c
load1/load5/load15
cpu_freq_min_mhz/cpu_freq_avg_mhz
raw_window_xy_p2p_m
filtered_window_xy_p2p_m
stationary_filtered_window_xy_p2p_m
map_quality_fitness
map_quality_jump_xy_m
map_quality_reason_code
map_quality_align_ms
map_quality_scan_age_sec
map_quality_odom_age_sec
map_quality_map_to_odom_delta_xy_m
map_quality_overlap_ratio
motion_factor
is_stationary
spike_candidate
sustained_motion
static_context_raw_xy_p2p_m
static_context_filtered_xy_p2p_m
filter_status_accepted
filter_status_alpha
filter_status_motion_factor
filter_status_recovered
```

判断规则：

- `motion_factor` 接近 0 且 `is_stationary=1` 时，静态漂移统计有效。
- 车在动时，不要把轨迹变化当成漂移。
- 温度、load、CPU 降频和 `stationary_filtered_window_xy_p2p_m` 同时上升，说明抖动可能和热/负载有关。
- `spike_candidate=1` 表示当前样本像静止尖峰，不能直接当作真实运动忽略。
- `sustained_motion=1` 才表示连续运动足够久，静态漂移统计应暂停。
- `static_context_*` 不会被单帧尖峰清空，适合排查“r2 没动但坐标突然跳了几厘米”。
- `health_events.jsonl` 只在 raw/filter/map/filter-status 出现跳变、拒绝、恢复或 bad fitness 时写入事件前后窗口摘要。
- `/map_lidar/quality` 的 `reason_code` 为 `0 accepted, 1 too_few_points, 2 not_converged, 3 bad_fitness`。
- `RC_MAP_QUALITY_OVERLAP=1` 可临时打开 scan/map 重合率诊断，默认保持 `0`，避免增加 r2 常态负载。
- 如果低负载无抖动，高负载有尖峰，优先关掉 VS Code、Chromium、QQ、Clash GUI、RViz、PlotJuggler 等非必要负载。
- `health_monitor_run.json` 启动即写，用于确认本轮 `RUN_REASON` 是 `manual`、`boot` 还是 `can_restart`。
- `health_monitor_stop.json` 只有正常收到停止信号并收尾时才出现。CAN 重启一般会保留 stop marker；直接断电再开机通常没有上一轮 stop marker。
- CSV 每行 flush，默认每 5 秒 fsync；summary 使用临时文件原子替换。直接断电时仍可能丢最后几秒数据，但不应影响已经落盘的大部分记录。
- 健康日志文件默认保留 7 天：`health_monitor.csv`、`health_monitor_summary.json`、`health_monitor_run.json`、`health_monitor_stop.json`、`health_events.jsonl`、`health_event_context.csv`。
- 原始运行日志目录也会做保守清理：`logs/autostart/run_*` 默认保留 30 天，并额外保留最近 10 次、`latest` 指向的当前运行、`latest_restart` 指向的最近重启日志。它不会删除 `maps/`、`logs/calibration/`、`report_assets/`，也不会按文件名去删 PCD 或校准报告。
- 每次一键启动会把原始日志清理结果写到当前运行目录的 `log_cleanup.log`。

可调参数：

```bash
RC_HEALTH_MONITOR_INTERVAL=0.2
RC_HEALTH_MONITOR_SUMMARY_INTERVAL=30.0
RC_HEALTH_MONITOR_FSYNC_INTERVAL=5.0
RC_HEALTH_MONITOR_WINDOW=10.0
RC_HEALTH_MONITOR_EVENT_PRE_SEC=3.0
RC_HEALTH_MONITOR_EVENT_POST_SEC=3.0
RC_HEALTH_MONITOR_SPIKE_RAW_STEP=0.02
RC_HEALTH_MONITOR_SPIKE_FILTERED_STEP=0.015
RC_HEALTH_MONITOR_SPIKE_MAP_JUMP=0.02
RC_HEALTH_MONITOR_SPIKE_FITNESS=0.02
RC_HEALTH_MONITOR_SUSTAINED_MOTION_SEC=2.0
RC_HEALTH_MONITOR_EVENT_CONTEXT_CSV=0
RC_HEALTH_MONITOR_RETENTION_DAYS=7
RC_AUTOSTART_LOG_RETENTION_DAYS=30
RC_AUTOSTART_LOG_KEEP_RECENT=10
RC_AUTOSTART_LOG_CLEANUP=1
RC_AUTOSTART_LOG_CLEANUP_DRY_RUN=1
RC_MAP_QUALITY_OVERLAP=0
```

低负载对照启动：

```bash
cd "$R2_WS"
./autostart/stop_all.sh
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
RC_AUTOSTART_TF_COMPARE=0 \
RC_AUTOSTART_LIDAR_XYZR=0 \
./autostart/start_all_terminals.sh
```

## 11. 常见故障排查

RViz 黑屏或 `Frame [camera_init] does not exist`：

```bash
ip -br link
ethtool "$R2_LIVOX_IFACE" | grep -i "Link detected"
tail -n 120 logs/autostart/latest/LIVOX.log
tail -n 120 logs/autostart/latest/FASTLIO.log
ros2 topic list | grep livox
```

优先确认网线和雷达供电，再看 IP 和 `MID360_config.json`。

CAN 灯不闪：

```bash
./autostart/can_setup.sh --check-only
ip -details -statistics link show type can
tail -n 120 logs/autostart/latest/CAN_SERIAL.log
```

如果没有 SocketCAN 接口，先处理 USB-CAN 枚举。若 TX 增加但接收端无反应，检查接收端波特率、终端电阻、CANH/CANL 和接收代码。

重定位输出接近 0 但车不在原点：

```bash
tail -n 160 logs/autostart/latest/MAP_LOCALIZER.log
ros2 topic echo /map_lidar/quality --once
```

不要使用 `RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1`。改用粗略初值或把车放回建图原点附近。

关闭终端后还有进程：

```bash
./autostart/stop_all.sh
ps aux | grep -Ei 'livox|fastlio|map_localizer|pose_filter|rc_serial|static_transform|plotjuggler' | grep -v grep
```

日志目录解释：

```text
logs/autostart/run_YYYYMMDD_HHMMSS_reason
```

其中 `HHMMSS` 是启动脚本创建该次运行日志的本机时间。

## 12. Git 提交和同步

迁移前保存 r2 当前状态：

```bash
cd "$R2_WS"
git status -sb
git diff --stat
```

迁移后只提交代码、脚本和说明文档。不要提交：

```text
note.md
logs/
maps/*.pcd
report_assets/
build/
install/
log/
```

提交示例：

```bash
git add autostart src readme.md R2_MIGRATION_RUNBOOK.md .gitignore
git restore --staged note.md 2>/dev/null || true
git commit -m "r2: migrate low-load relocalization startup flow"
git pull --rebase origin main
git push origin main
```

如果远端拒绝 push，先 `git pull --rebase origin main`，解决冲突后再 push。不要用 `git reset --hard` 或强推，除非用户明确要求。

维护提醒：

- 以后在当前仓库更新 README 的同时，也要同步维护 `R2_MIGRATION_RUNBOOK.md`。
- 如果改的是运行默认值、启动命令、PlotJuggler topic、CAN 坐标来源、滤波参数、健康日志字段、地图路径或自启策略，必须把 r2 的复制步骤也写进本文档。
- 如果某次修改只影响实验报告、临时日志或本机私有笔记，可以不写进本文档。

## 13. 最终验收清单

完成后逐项验证：

```bash
cd "$R2_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

./autostart/stop_all.sh
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
./autostart/start_all_terminals.sh

sleep 30
ros2 topic echo /robot/pose --once
ros2 topic echo /robot/pose_filtered --once
ros2 topic echo /map_lidar/quality --once
ros2 topic echo /lidar/pose_filter/status --once
tail -n 120 logs/autostart/latest/CAN_SERIAL.log
ls -lh logs/autostart/latest/health_monitor.*
```

验收通过标准：

- 雷达网口 `Link detected: yes`。
- Livox topic、FAST-LIO odometry、`camera_init -> body` TF 正常。
- `MAP_LOCALIZER` 发布质量 topic，且没有持续 rejected。
- `/robot/pose_filtered` 连续刷新。
- CAN 使用 `/robot/pose_filtered`，有接收端时 TX 计数增长。
- 健康日志存在，能记录温度、load、CPU 频率、地图质量和静止漂移。
- `./autostart/stop_all.sh` 能清掉残留雷达、定位、滤波、CAN 和 GUI 调试进程。
