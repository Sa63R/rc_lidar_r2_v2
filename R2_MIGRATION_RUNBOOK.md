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

当前 Orange Pi 的 CAN 激活已经包含在一键启动里：`start_all_terminals.sh -> CAN_SERIAL -> serial_driver.sh -> can_setup.sh -> rc_serial_driver`。迁移到 r2 时也应保持这个结构，不要另写一套独立 CAN 启动流程。`can_setup.sh` 会自动检测第一个 `can*` / `slcan*`，并在有免密权限或 udev 自动配置时设置 `1000000` bit/s、`restart-ms 100`、`UP`。

真正免密一键启动依赖一次性系统配置：

```bash
cd "$R2_WS"
./autostart/install_one_click_network.sh
```

这个脚本会安装 USB-CAN udev 规则。之后 USB-CAN 枚举成 `can*` 时会由系统自动拉起 CAN；一键启动时 `can_setup.sh` 只需要检查并复用该接口。如果 r2 没有运行这个脚本，也没有免密 sudo，那么 `can0` 已经 `UP` 时可以继续启动，但 `can0` 存在却未 `UP` 时 `CAN_SERIAL` 会停止，避免卡在 sudo 密码。

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

如果 `lsusb` 显示 `0483:df11 STMicroelectronics STM Device in DFU Mode` 或 `STM32 BOOTLOADER`，说明 USB-CAN 处在 BOOT/DFU 模式，不会生成 `can0/can1`。把开关拨回正常 CAN 模式后必须重新插拔或复位，期望看到 `0c72:000c PEAK System PCAN-USB / XCAN-USB`，随后 `ip -details link show type can` 应出现 `can0`。

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
RC_CAN_REQUIRE_READY=1
RC_CAN_READY_QUALITY_TOPIC=/map_lidar/quality
RC_CAN_READY_MAX_FITNESS=0.05
RC_CAN_READY_MIN_ACCEPTS=5
```

`RC_CAN_REQUIRE_READY=1` 表示 CAN 发送前必须看到 `/map_lidar/quality` 连续 accepted/converged、fitness 达标并且没有超时。迁移到 R2 时建议保留这个门控，避免重定位还没稳定时把早期假坐标发给电控。只有在临时排查 CAN 链路本身时，才考虑短时间设为 `0`。

临时回退旧 TF 链路：

```bash
RC_CAN_POSE_SOURCE=tf ./autostart/start_all_terminals.sh
```

## 5. 迁移启动策略和开机自启

安装自启入口时，只安装 user systemd。不要默认安装 XDG 桌面自启；它依赖图形登录会话，而且会让同一台车上出现两个开机入口。当前 Orange Pi 生产策略已经移除已安装的桌面自启，只保留 `rc-lidar-autostart.service`。`boot_start.sh` 仍保留 `boot_id` 防重复机制，作为兜底。

```bash
cd "$R2_WS"
./autostart/install_user_systemd_autostart.sh
systemctl --user status rc-lidar-autostart.service
loginctl show-user "$(id -un)" -p Linger
```

如果 r2 之前装过桌面 XDG 自启，先删除已安装入口：

```bash
rm -f "$HOME/.config/autostart/rc_lidar_autostart.desktop"
systemctl --user daemon-reload
```

R2 上不要直接复制 Orange Pi 的 systemd 用户目录；只运行仓库里的安装脚本。`install_user_systemd_autostart.sh` 应尝试启用当前用户 linger。若 R2 没有外接显示器或没有自动登录图形桌面，必须确认 `Linger=yes`，这样 user systemd 服务才能在纯开机阶段启动。若不是 `yes`，执行：

```bash
loginctl enable-linger "$(id -un)"
```

然后用下面命令确认自启入口有没有被调用：

```bash
tail -n 80 logs/autostart/boot_attempts.log
tail -n 120 logs/autostart/xdg_autostart.log
cat logs/autostart/boot_start_state.env
find logs/autostart -maxdepth 1 -type d -name 'run_*' -printf '%TY-%Tm-%Td %TH:%TM:%TS %f\n' | sort -r | head
```

注意：开机后系统时间可能先不准，之后被 NTP/chrony 校正。排查日志先后时，不要只看 `run_YYYYMMDD_HHMMSS` 这一段文件名；当前 run 目录会追加 `boot_id` 短码和启动脚本 PID，例如 `run_20260621_172000_boot_517066fa_1234`，用来避免连续强关重启时日志目录碰撞。同时看 `start_summary.txt` 里的 `boot_id`、`uptime_sec`、`elapsed_sec` 和 `epoch_sec`。

当前 Orange Pi 工程还有一个重定位 ready 提示节点 `READY_NOTIFY`。迁移到 R2 时也建议保留：它订阅 `/robot/pose_filtered` 和 `/map_lidar/quality`，当重定位连续 accepted/converged 且 `fitness` 达标后写：

```text
logs/autostart/<run>/relocalization_ready.json
```

如果 R2 有图形桌面和终端，它还会弹出“RC LiDAR 重定位完成，现在可以移动小车”的小窗口。这个节点不参与定位计算，ready 后自动退出，常态开销近似为零。R2 上需要按现场情况复核这些阈值：

```bash
RC_READY_NOTIFY_STABLE_SEC=3.0
RC_READY_NOTIFY_TIMEOUT_SEC=120.0
RC_READY_NOTIFY_MAX_FITNESS=0.05
RC_READY_NOTIFY=1          # 写 ready json
RC_READY_NOTIFY_POPUP=0    # 无显示器/生产开机默认不弹窗口
```

先做 dry-run：

```bash
cd "$R2_WS"
RC_AUTOSTART_DRY_RUN=1 ./autostart/start_all_terminals.sh
```

dry-run 只用于检查启动项，不应覆盖真实运行指针。当前约定是：真实运行读 `logs/autostart/latest`；dry-run 预检查读 `logs/autostart/latest_dry_run`。迁移到 R2 时也要保持这个规则，避免把预检查目录误判成实测日志。

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
TF_MAP_CAMERA_INIT 或 MAP_LOCALIZER 发布的 map -> camera_init
MAP_LOCALIZER
POSE_INTERFACE
POSE_FILTER
HEALTH_MONITOR
CAN_SERIAL
```

当前 Orange Pi 策略里，`RC_AUTOSTART_HEADLESS=auto` 默认让核心雷达/定位/CAN/健康日志节点都用后台 headless 方式启动；手动启动时 `RC_AUTOSTART_PLOTJUGGLER=auto` 仍会打开 PlotJuggler。迁移到 R2 时也建议采用这个策略：正常调试启动和低负载启动保持同一条核心链路，只用 `RC_AUTOSTART_PLOTJUGGLER=0/1` 决定是否打开 PlotJuggler。若需要旧式“每个节点一个终端窗口”调试，再显式设置：

```bash
RC_AUTOSTART_HEADLESS=0 ./autostart/start_all_terminals.sh
```

停止所有残留进程：

```bash
cd "$R2_WS"
./autostart/stop_all.sh
```

不要只靠关闭终端窗口停止程序。headless、开机自启和 CAN 重启会把进程放到后台。

确认没有桌面 XDG 自启入口：

```bash
cd "$R2_WS"
rm -f "$HOME/.config/autostart/rc_lidar_autostart.desktop"
systemctl --user daemon-reload
```

检查自启目录里没有 RC LiDAR 的桌面入口：

```bash
ls -l "$HOME/.config/autostart"
grep -R "RC_LIDAR\\|RC Lidar" "$HOME/.config/autostart" -n || true
```

开机自启默认应满足：

```text
RC_LIDAR_MODE=relocalization
RC_LIDAR_MAP_PATH=<R2 verified small-field map path>
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1
RC_AUTOSTART_HEADLESS=1
RC_AUTOSTART_PLOTJUGGLER=0
RC_CAN_POSE_SOURCE=pose_topic
RC_CAN_POSE_TOPIC=/robot/pose_filtered
RC_POSE_FILTER_STABLE=0
RC_HEALTH_MONITOR=1
RC_READY_NOTIFY=1
RC_READY_NOTIFY_POPUP=0
```

CAN 重启脚本也应保持同样默认策略：

```bash
grep -n "RC_LIDAR_MODE\\|RC_CAN_POSE_TOPIC\\|RC_HEALTH_MONITOR" autostart/restart_lidar_stack.sh
```

Orange Pi 当前生产默认已经固定为小场地图 `/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260625_064655.pcd`，不开 PlotJuggler，ready 只写 json 不弹窗口，并默认不启动 stable 候选滤波，CAN 发 normal `/robot/pose_filtered`。R2 不能照搬这个绝对路径，但应该采用同样策略：把 `boot_start.sh`、`rc-lidar-autostart.service` 和 `restart_lidar_stack.sh` 的默认 `RC_LIDAR_MAP_PATH` 都指向 R2 自己验证过的小场 PCD，并保持 `RC_POSE_FILTER_STABLE=0`；开机入口只保留 user systemd，只有需要曲线对比时才手动设置 `RC_AUTOSTART_PLOTJUGGLER=1` 或 `RC_POSE_FILTER_STABLE=1`。

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

如果保存脚本长时间停在 `waiting for /map_save service`，不要先改保存脚本，按下面顺序查：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list | grep laser_mapping
ros2 service type /map_save
tail -n 80 logs/autostart/latest/FASTLIO.log
tail -n 80 logs/autostart/latest/LIVOX.log
tail -n 80 logs/autostart/latest/POSE_INTERFACE.log
```

正常时 `/laser_mapping` 应存在，`/map_save` 应返回 `std_srvs/srv/Trigger`。如果 R2 的 `POSE_INTERFACE.log` 报 `publish_rate` 参数类型错误，检查 `pose_from_tf` 是否已经同步当前 Orange Pi 的兼容修复：`publish_rate` 要允许整数/小数，启动脚本默认传 `50.0`。如果 FAST-LIO 或 Livox 已退出，先用 `./autostart/stop_all.sh` 清理后重新启动建图，再检查服务。

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

如果 R2 的比赛区域只是大场馆的一小块，不要直接拿整张大场馆 PCD 做生产重定位。当前 Orange Pi 已验证的可用方案是：只绕比赛小场地建一张小 PCD 地图，不裁剪，然后从建图原点附近启动。迁移到 R2 时优先复刻这个流程。

Orange Pi 可用检查点：

```text
map: /home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260624_190419.pcd
mode: RC_LIDAR_MODE=relocalization
initialization: RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1
note: 小场地直接建图，不使用 pcd_prepare_local_map 裁剪
```

如果 R2 只有一整张大场馆 PCD，且暂时无法重新绕小场地建图，才考虑裁剪出比赛区及少量缓冲区：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run lidar_map_localization pcd_prepare_local_map \
  --input maps/mid360_map.pcd \
  --output maps/competition_local.pcd \
  --leaf 0.10 \
  --min-x -4.0 --max-x 4.0 \
  --min-y -3.0 --max-y 5.0 \
  --margin 2.0
```

也可以用圆形区域：

```bash
ros2 run lidar_map_localization pcd_prepare_local_map \
  --input maps/mid360_map.pcd \
  --output maps/competition_local.pcd \
  --leaf 0.10 \
  --center-x 0.0 --center-y 0.0 --radius 6.0 \
  --margin 2.0
```

裁剪坐标必须按 R2 实际 PCD 的 FAST-LIO map 坐标确认，不要照搬 Orange Pi 的示例数值。工具不会覆盖已有输出；路径冲突时会自动加时间戳，并写出 `*.report.json` 记录点数和裁剪范围。

注意：裁剪工具保留为备用，但本次 Orange Pi 可用版本没有用到它。R2 同步时优先验证“小场直接建图”的链路。

## 7. 重定位验证

确认地图存在：

```bash
ls -lh maps/mid360_map.pcd
```

如果机器人放在建图开始原点附近：

```bash
cd "$R2_WS"
RC_LIDAR_MODE=relocalization \
RC_LIDAR_MAP_PATH="$R2_WS/maps/<r2_verified_small_field_map>.pcd" \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
./autostart/start_all_terminals.sh
```

如果暂时没有局部地图，可以先不设置 `RC_LIDAR_MAP_PATH`，默认使用 `maps/mid360_map.pcd`。一旦大场馆全图出现“识别不到自己在哪里”或局部最优跳变，优先回到局部地图和固定起点策略，而不是盲目调 NDT 参数。

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
- `CAN_SERIAL.log` 显示 ready gate enabled；`/map_lidar/quality` 未稳定时 CAN 应暂停发送，稳定后 TX 计数才持续增加。

Orange Pi Rc 4.4 起，`/robot/pose` 和 `/robot/pose_filtered` 默认使用本机发布时刻作为 header stamp，避免 `/map_lidar/odom` 的旧 stamp 让 CAN 和健康日志误判 topic stale。迁移到 R2 时建议保留这个默认值；只有为了复现实验才回退：

```bash
RC_ROBOT_POSE_STAMP_MODE=input
RC_POSE_FILTER_STAMP_MODE=input
```

同一版给 normal/beta/stable 滤波增加 `/map_lidar/quality` 保守门控。默认只在本帧质量明确 rejected/not converged/bad fitness，或 `map_quality_jump_xy_m > 0.08m` 时 hold 上一帧；如果 quality topic 不存在或超时，门控 fail-open，不阻断定位。迁移时先保留这些默认值，再按 r2 现场日志调小或调大：

```bash
RC_POSE_FILTER_QUALITY_GATE=1
RC_POSE_FILTER_QUALITY_TIMEOUT=1.0
RC_POSE_FILTER_QUALITY_MAX_JUMP=0.08
RC_POSE_FILTER_QUALITY_MAX_FITNESS=0.20
```

Orange Pi Rc 4.6 起，yaw 使用独立低延迟 EMA，避免无头车自转后航向角被 XY 静态强滤波拖慢。迁移到 R2 时建议先保留默认：

```bash
RC_POSE_FILTER_YAW_ALPHA=0.85
```

这里的含义是：XY 继续由 `RC_POSE_FILTER_STATIC_ALPHA`、`RC_POSE_FILTER_MOVING_ALPHA` 和速度阈值控制，静止时压抖动；yaw 单独用 `0.85` 快速跟随。若 R2 现场 yaw 仍滞后，可临时调到 `1.0`；若 yaw 抖动明显，再回调到 `0.6~0.8`。beta/stable 候选分别可以用 `RC_POSE_FILTER_BETA_YAW_ALPHA`、`RC_POSE_FILTER_STABLE_YAW_ALPHA` 覆盖。

Orange Pi Rc 4.7 又增加了一个更直接的无头车调试开关：

```bash
RC_POSE_FILTER_USE_RAW_YAW=0
```

默认 `0`，yaw 仍按 `RC_POSE_FILTER_YAW_ALPHA` 轻滤波，并随整帧质量门控一起 hold。设置为 `1` 时，`/robot/pose_filtered` 的 yaw 直接使用 `/robot/pose` 原始 yaw；即使 XY 因 `quality_gate` 被 hold，yaw 也继续更新。迁移到 R2 时这个开关不要默认打开，只有在确认 raw yaw 曲线稳定、但 filtered yaw 被 quality gate 卡死造成无头车转向延迟时再打开。beta/stable 候选可以分别用 `RC_POSE_FILTER_BETA_USE_RAW_YAW`、`RC_POSE_FILTER_STABLE_USE_RAW_YAW` 覆盖。

Orange Pi Rc 4.8 进一步把低延迟 yaw 放到 `/robot/pose` 上游，而不是只在滤波器里绕过 yaw。重定位模式下 `pose_from_odom` 仍用 `/map_lidar/odom` 的地图重定位结果提供 XY，但 yaw 默认使用：

```text
fused_yaw = latest_map_yaw_reference + (fastlio_yaw_now - fastlio_yaw_at_reference)
```

也就是用地图 yaw 保证全局方向，用 FAST-LIO `/Odometry` 高频 yaw 增量降低无头车旋转延迟。R2 迁移时建议保留默认：

```bash
RC_ROBOT_POSE_YAW_FUSION=1
RC_ROBOT_POSE_FAST_ODOM_TOPIC=/Odometry
RC_ROBOT_POSE_YAW_FUSION_MAP_CORRECTION_ALPHA=0.25
```

如果 R2 现场发现 fused yaw 比旧 map yaw 更抖，可临时回退：

```bash
RC_ROBOT_POSE_YAW_FUSION=0 ./autostart/start_all_terminals.sh
```

同时观察 `/robot/yaw_fusion/status`：`data[1]` 为 ready，`data[3]` 为 FAST-LIO yaw 年龄，`data[4]` 为最近 map yaw 年龄，`data[8]` 为 fast yaw 增量。FAST-LIO 年龄长期变大时，说明低延迟 yaw 输入本身没跟上，不能继续调滤波来解决。

当前 Orange Pi Rc 2.9 滤波已参考 r2 normal/beta 方案，但默认采用更保守的生产策略：

```bash
# 默认：只启动 normal，CAN 发 /robot/pose_filtered
./autostart/start_all_terminals.sh

# 手动对比 beta，但 CAN 仍发 normal
RC_POSE_FILTER_BETA=1 ./autostart/start_all_terminals.sh

# 确认 beta 更适合现场后，才让 CAN 发 beta
RC_POSE_FILTER_BETA=1 \
RC_POSE_FILTER_SELECTED=beta \
./autostart/start_all_terminals.sh
```

Rc 3.9 又增加 stable 候选滤波，用于处理静止时 map localizer 输出 1-4cm 短时跳点、normal 被瞬时速度误判成真实运动的问题：

```bash
# 手动对比 stable，但 CAN 仍发 normal
RC_POSE_FILTER_STABLE=1 ./autostart/start_all_terminals.sh

# 确认 stable 更适合现场后，才让 CAN 发 stable
RC_POSE_FILTER_STABLE=1 \
RC_POSE_FILTER_SELECTED=stable \
./autostart/start_all_terminals.sh
```

迁移到 R2 时优先保留这个保守策略：beta/stable 先用于 PlotJuggler 和健康日志对比，不默认进入电控生产链路。`RC_POSE_FILTER_SELECTED=beta` 必须和 `RC_POSE_FILTER_BETA=1` 同时设置；`RC_POSE_FILTER_SELECTED=stable` 必须和 `RC_POSE_FILTER_STABLE=1` 同时设置，否则启动脚本应直接报错，避免 CAN 读取不存在的候选 topic。

## 8. TF、外参和自转校准

当前静态 TF 结构：

```text
map -> robot_origin
body -> robot_base
body_highrate -> robot_base_highrate
```

重定位模式下 `map_localizer` 发布 `map -> camera_init`；mapping/tracking 模式需要发布 identity `map -> camera_init`。`map` 是 PCD 地图坐标，Y 正方向作为无头车世界正方向；`robot_origin` 是机器人业务原点。

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
ros2 run tf2_ros tf2_echo map robot_origin
ros2 run tf2_ros tf2_echo body robot_base
ros2 run tf2_ros tf2_echo body_highrate robot_base_highrate
ros2 run tf2_ros tf2_echo robot_origin robot_base_highrate
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
- 等重定位 ready，或确认 `robot_origin -> robot_base_highrate` 能被查询。
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

当前 PlotJuggler 自动启动策略不是“强行画好所有曲线”，而是做低风险 topic 自动选择：

```text
1. 最多等待关键 ROS topic 出现 16 秒。
2. 写入 PlotJuggler 用户配置里的 DataStreamROS2/default_topics 和 selected_topics。
3. 默认不加载 XML 布局，只依赖用户配置预选 topic，避免 PlotJuggler 版本差异导致崩溃。
4. 用 xdotool 自动确认 Warning、Start Streaming?、Topic Selector 等弹窗。
```

等待时间可用 `PLOTJUGGLER_WAIT_SECONDS` 覆盖；每次 ROS discovery 默认等待 `2s`，可用 `PLOTJUGGLER_TOPIC_SPIN_TIME` 覆盖。如果日志里持续显示一整组 topic 都在等待，先看是否有 `ros2 topic list failed`；这通常是 ROS graph 查询暂时失败或 daemon 状态异常，不等于每个 topic 都真的未发布。当前脚本等待 topic 时优先使用 `ros2 topic list --no-daemon --spin-time 2`，避免 ROS daemon 缓存或 `rclpy.ok()` 错误导致误判。

r2 上需要安装：

```bash
sudo apt install -y ros-humble-plotjuggler ros-humble-plotjuggler-ros xdotool
```

脚本会更新这个文件：

```text
~/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini

[DataStreamROS2]
default_topics=...
selected_topics=...
```

重定位模式下应默认预选这些 topic：

```text
/robot/pose
/robot/pose_filtered
/map_lidar/quality
/lidar/pose_filter/status
/robot/yaw_fusion/status
/tf_compare/robot_origin_to_robot_base
/tf_compare/robot_origin_to_robot_base_highrate
/tf_compare/map_to_body
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
grep -n "selected_topics" "$HOME/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini"
```

如果仍然看不到 `/robot/pose_filtered`，确认 topic 本身已经存在：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list --no-daemon --spin-time 2 | grep robot/pose
ros2 topic info /robot/pose_filtered --verbose --no-daemon
tail -n 120 logs/autostart/latest/PLOTJUGGLER.log
```

如果 PlotJuggler 已经订阅 `/robot/pose_filtered`，但左侧没有 `/robot/pose` 或 `/robot/pose_filtered` 曲线，继续检查位姿接口本身，而不是只重启 PlotJuggler：

```bash
ros2 topic info /robot/pose --verbose --no-daemon
tail -n 120 logs/autostart/latest/POSE_INTERFACE.log
```

当前 Orange Pi 的重定位生产链路默认使用 `RC_ROBOT_POSE_SOURCE=odom`：`POSE_INTERFACE` 订阅 `/map_lidar/odom`，在同一帧定位结果上加雷达到机器人中心外参，并发布 `/robot/pose`。这保留无头车 TF 树，但避免用 TF buffer 直接拼接 `robot_origin -> robot_base_highrate` 时混合低频 `map -> camera_init` 和高频 `body_highrate`。

只有明确要诊断旧 TF 拼接路径时，才临时设置：

```bash
RC_ROBOT_POSE_SOURCE=tf ./autostart/start_all_terminals.sh
```

如果 PlotJuggler 仍然要求手点确认，检查：

```bash
command -v xdotool
echo "$DISPLAY"
tail -n 120 logs/autostart/latest/PLOTJUGGLER.log
```

如果 `xdotool` 不存在，PlotJuggler 仍会启动，但需要手动点击确认窗口。PlotJuggler ROS2 插件没有可靠的完全静默订阅参数，本工程采用“启动前写入 `default_topics/selected_topics` + 自动确认弹窗”的方案。当前 Orange Pi 上 PlotJuggler 3.15 加载轻量 XML 后出现过内部崩溃，所以默认不加载 XML；R2 也先沿用“不加载布局，只预选 topic”的策略。

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
yaw_fusion_ready
yaw_fusion_fast_age_sec
yaw_fusion_map_age_sec
yaw_fusion_delta_yaw_deg
```

判断规则：

- `motion_factor` 接近 0 且 `is_stationary=1` 时，静态漂移统计有效。
- 车在动时，不要把轨迹变化当成漂移。
- 温度、load、CPU 降频和 `stationary_filtered_window_xy_p2p_m` 同时上升，说明抖动可能和热/负载有关。
- `spike_candidate=1` 表示当前样本像静止尖峰，不能直接当作真实运动忽略。
- `sustained_motion=1` 才表示连续运动足够久，静态漂移统计应暂停。
- `static_context_*` 不会被单帧尖峰清空，适合排查“r2 没动但坐标突然跳了几厘米”。
- `health_events.jsonl` 只在 raw/filter/map/filter-status 出现跳变、拒绝、恢复或 bad fitness 时写入事件前后窗口摘要。
- `yaw_fusion_fast_age_sec` 长期很小但 `yaw_fusion_map_age_sec` 变大，说明 FAST-LIO 高频 yaw 正常、地图更新较慢；此时 fused yaw 应比纯 `/map_lidar/odom` yaw 更低延迟。
- `/map_lidar/quality` 的 `reason_code` 为 `0 accepted, 1 too_few_points, 2 not_converged, 3 bad_fitness`。
- `RC_MAP_QUALITY_OVERLAP=1` 可临时打开 scan/map 重合率诊断，默认保持 `0`，避免增加 r2 常态负载。
- 如果低负载无抖动，高负载有尖峰，优先关掉 VS Code、Chromium、QQ、Clash GUI、RViz、PlotJuggler 等非必要负载。
- Codex 现场调代码时通常必须开 VS Code 和 Clash，有时还会开 QQ、浏览器或 PlotJuggler；这个状态会显著提高 CPU/温度负载，不能直接代表开机 headless 低负载状态。R2 复刻时，判断滤波、TF 或地图质量之前，先做一轮不开 GUI 调试工具的 headless 日志对照。
- 如果 `/robot/pose` 原始位姿已经出现厘米级跳变，并且 `/map_lidar/quality` 的 `map_quality_jump_xy_m` 或 fitness 同步变差，问题优先看地图匹配/初值/场地结构/负载调度，滤波只能后处理，不能从根上修正错误局部匹配。
- 如果健康日志 5Hz 且 `health_events.jsonl` 高频刷事件，在高负载现场它本身也可能变成可见开销。排查性能时可临时设置 `RC_HEALTH_MONITOR_INTERVAL=1.0`，并关闭 beta/stable 候选，只保留 normal 链路做基线。
- `health_monitor_run.json` 启动即写，用于确认本轮 `RUN_REASON` 是 `manual`、`boot` 还是 `can_restart`。
- `health_monitor_stop.json` 只有正常收到停止信号并收尾时才出现。CAN 重启一般会保留 stop marker；直接断电再开机通常没有上一轮 stop marker。
- CSV 每行 flush，默认每 5 秒 fsync；summary 使用临时文件原子替换。直接断电时仍可能丢最后几秒数据，但不应影响已经落盘的大部分记录。
- 健康日志文件默认保留 7 天：`health_monitor.csv`、`health_monitor_summary.json`、`health_monitor_run.json`、`health_monitor_stop.json`、`health_events.jsonl`、`health_event_context.csv`。

低负载基线对照命令：

```bash
cd "$R2_WS"
./autostart/stop_all.sh

RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
RC_POSE_FILTER_STABLE=0 \
RC_HEALTH_MONITOR_INTERVAL=1.0 \
RC_LIDAR_MAP_PATH="<R2 verified small-field map path>" \
./autostart/start_all_terminals.sh
```

这条命令用于排查“normal 滤波本身是否可靠”和“地图重定位源头是否已经在跳”。它不降低雷达频率，只减少调试节点和日志压力。
- 原始运行日志目录也会做保守清理：`logs/autostart/run_*` 默认保留 30 天，并额外保留最近 10 次、`latest` 指向的当前真实运行、`latest_dry_run` 指向的最近 dry-run 预检查、`latest_restart` 指向的最近重启日志。它不会删除 `maps/`、`logs/calibration/`、`report_assets/`，也不会按文件名去删 PCD 或校准报告。
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

如果没有 SocketCAN 接口，先处理 USB-CAN 枚举。若看到 `STM32 BOOTLOADER/DFU`，先拨回正常 CAN 模式并重新插拔。若 TX 增加但接收端无反应，检查接收端波特率、终端电阻、CANH/CANL、接收代码以及接收端是否处于 silent/listen-only 模式。

健康日志或 ROS Python 节点报 `rclpy._rclpy_pybind11`：

```bash
command -v python3
python3 --version
echo "$CONDA_PREFIX"
tail -n 120 logs/autostart/latest/HEALTH_MONITOR.log
```

ROS 2 Humble 应使用系统 Python 3.10。如果 r2 机器装过 conda/miniconda，且终端默认进入 `(base)` 或 `python3` 指向 conda，就可能导致 `rclpy` 导入失败。迁移时保留当前仓库的 `autostart/common_env.sh` conda 隔离逻辑；它会在雷达启动脚本里清理 conda 环境并优先使用系统 Python。若新开终端反复提示 `Ignoring configuration file ~/.condarc`，通常是用户目录的 `.condarc` 损坏；先备份，再写入最小合法配置 `auto_activate_base: false`，不要在 `.bashrc` 每次启动时反复执行 `conda config --set auto_activate false`。

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
- 如果改的是地图准备、局部地图裁剪、重定位初值策略或 CAN READY 门控，也必须同步到本文档；这些是 R2 在大场地复刻时最容易踩坑的部分。
- 如果后续确认了某个现场可用地图/启动组合，也要写清楚它是否真的用了裁剪；Rc 3.2 的结论是小场直接建图可用，裁剪没用到。
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
# 如果手动启用了 RC_POSE_FILTER_BETA=1：
ros2 topic echo /robot/pose_filtered_beta --once
# 如果手动启用了 RC_POSE_FILTER_STABLE=1：
ros2 topic echo /robot/pose_filtered_stable --once
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
- `POSE_INTERFACE.log` 正常应显示从 `/map_lidar/odom` 发布 `/robot/pose`。如果看到 `TF robot_origin->robot_base_highrate`，说明当前启动用了 `RC_ROBOT_POSE_SOURCE=tf` 回退路径；该路径只用于对比诊断，不建议作为 R2 生产默认。
- `POSE_FILTER.log` 不应出现 `ValueError: Logger severity cannot be changed between calls`。如果出现，说明 r2 还没有同步 Rc 3.8 的滤波日志修复；该问题会让 `/robot/pose_filtered` 运行一段时间后停止发布。
- 如果启用 beta，对比 `/robot/pose_filtered_beta`，确认静止尖峰更少且真实连续运动不会长期 hold。
- 如果启用 stable，对比 `/robot/pose_filtered_stable`，确认静止 1-4cm 短时跳点不会被快速跟随，同时慢速连续移动和绕点转圈不会出现平台/锯齿。
- CAN 使用 `/robot/pose_filtered`，有接收端时 TX 计数增长。
- 健康日志存在，能记录温度、load、CPU 频率、地图质量和静止漂移。
- `./autostart/stop_all.sh` 能清掉残留雷达、定位、滤波、CAN 和 GUI 调试进程。
