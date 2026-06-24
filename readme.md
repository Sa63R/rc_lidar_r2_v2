# RC LIDAR ROS 2 工作区编译和运行说明

本文记录当前 `src/` 工作区的编译环境、构建命令、运行顺序、硬件前置条件，以及为了适配当前环境做过的源码修改。

## 当前已验证环境

- 系统：Ubuntu 22.04.5 LTS, aarch64
- ROS：ROS 2 Humble
- 工作区路径：`/home/orangepi/RC_LIDAR_NOW`
- 已存在关键工具：`colcon`、`ros2`、`rosdep`
- 已安装 PlotJuggler：`ros-humble-plotjuggler`、`ros-humble-plotjuggler-ros`
- 已存在 Livox-SDK2：
  - `/usr/local/include/livox_lidar_api.h`
  - `/usr/local/include/livox_lidar_def.h`
  - `/usr/local/lib/liblivox_lidar_sdk_shared.so`
  - `/usr/local/lib/liblivox_lidar_sdk_static.a`

当前机器上 `rosdep` 还没有初始化，且当前用户没有免密 sudo，所以部分系统级安装需要手动输入 sudo 密码。PlotJuggler 自动确认弹窗需要 `xdotool`。

## 包结构

`src/` 下包含 7 个 ROS 2 包：

- `livox_ros_driver2`：Livox 雷达驱动，发布 `/livox/lidar` 和 `/livox/imu`
- `fast_lio`：FAST-LIO 建图定位节点
- `lidar_map_localization`：读取保存的 PCD 地图，用 NDT/ICP 把 FAST-LIO 当前点云重定位到地图
- `lidar_localization`：把 `/Odometry` 或 `/map_lidar/odom` 转成稳定的机器人中心点位姿 `/robot/pose`，同时保留旧兼容名 `/lidar/pose`
- `my_static_tf`：静态 TF launch
- `fastlio_world_init`：根据 FAST-LIO 初始 TF 生成 world 初始化 TF，并发布 TF 对比曲线 topic 的 Python 节点
- `rc_serial_driver`：读取 TF 并通过 SocketCAN 发送位姿，同时监听 CAN 重启命令

## 新环境依赖安装

在一台新的 Ubuntu 22.04 + ROS 2 Humble 机器上，建议先安装基础依赖：

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
  libpcl-dev libeigen3-dev libboost-all-dev libapr1-dev
```

如果 `rosdep` 没初始化：

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`livox_ros_driver2` 还需要 Livox-SDK2。如果 `/usr/local/lib/liblivox_lidar_sdk_shared.so` 不存在，需要先安装 SDK：

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

## 编译

每次打开新终端，先进入工作区并加载 ROS：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
source /opt/ros/humble/setup.bash
```

编译全部包：

```bash
colcon build --symlink-install \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DROS_EDITION=ROS2 \
  -DHUMBLE_ROS=humble
```

编译成功后加载本工作区：

```bash
source install/setup.bash
```

本次验证结果：

```text
Summary: 7 packages finished
```

可执行文件检查：

```bash
for p in fast_lio livox_ros_driver2 lidar_map_localization lidar_localization rc_serial_driver fastlio_world_init my_static_tf; do
  echo "== $p =="
  ros2 pkg executables "$p"
done
```

当前验证到的可执行文件：

```text
fast_lio fastlio_mapping
lidar_map_localization map_localizer
lidar_localization pose_from_odom
lidar_localization print_lidar_pose
lidar_localization print_xy_yaw
livox_ros_driver2 livox_ros_driver2_node
rc_serial_driver rc_serial_driver_node
fastlio_world_init lidar_tf_offset
fastlio_world_init robot_tf_offsets
fastlio_world_init rotation_extrinsic_calibrator
fastlio_world_init world_from_fastlio_init
```

## 运行前硬件配置

### Livox MID360 网口

当前配置文件是：

```text
src/livox_ros_driver2/config/MID360_config.json
```

其中默认：

- 主机 IP：`192.168.1.5`
- 雷达 IP：`192.168.1.124`

当前机器实际活动网口是 `192.168.178.148/24`，不在 `192.168.1.0/24` 网段，所以启动 Livox 驱动时出现过：

```text
bind failed
Failed to init livox lidar sdk.
```

这不是编译问题，是网口 IP 和配置文件不匹配。运行前需要把接雷达的网口配置到 `192.168.1.5/24`，或者把 `MID360_config.json` 里的 host IP 改成实际雷达网段内的本机 IP。

当前 Orange Pi 上确认到 MID360 在 `eth0` 上，`arping -I eth0 192.168.1.124` 可以收到雷达回应。不要把同一个 `192.168.1.5/24` 同时配置到多个网口；否则 Linux 可能把去雷达的路由发到错误网口，导致 Livox 驱动启动但 `/livox/lidar` 和 `/livox/imu` 没有 publisher。

一键启动时 `autostart/livox.sh` 会先调用 `autostart/setup_livox_net.sh`，默认把 `eth0` 配置为 `192.168.1.5/24` 并用 ARP 探测 `192.168.1.124`，成功后才启动 Livox 驱动。如果已经在系统网络管理里做了永久配置，可以用下面的环境变量跳过自动配置：

```bash
RC_AUTOSTART_SETUP_LIVOX_NET=0 ./autostart/start_all_terminals.sh
```

常见错误：如果 RViz 里显示 `Frame [camera_init] does not exist`，画面全黑，并且 `world_tf.log` 一直停在 `Waiting for TF camera_init -> body`，优先不要改 TF。先检查 Livox 是否真的起来了。一次实际排查中，根因只是雷达网线没插好，`eth0` 虽然已经有 `192.168.1.5/24`，但链路状态是 `NO-CARRIER`，`ethtool` 显示 `Link detected: no`，所以 Livox 驱动没有启动，FAST-LIO 也就不会发布 `camera_init`。

快速检查命令：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
ip -br link show eth0
ethtool eth0 | grep -i "Link detected"
tail -n 80 logs/autostart/latest/LIVOX.log
```

正常情况下应看到 `Link detected: yes`，并且 `arping -I eth0 -c 3 192.168.1.124` 能收到雷达回应。如果看到 `NO-CARRIER` 或 `Link detected: no`，这是物理链路问题，优先检查雷达供电、网线、交换机/转接盒、网口是否插在 `eth0`。插好后建议清理旧进程再重启：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
./autostart/stop_all.sh
RC_LIDAR_MODE=mapping ./autostart/start_all_terminals.sh
```

为了保证开机后一键启动时不再要求输入 sudo 密码，先运行一次：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
./autostart/install_one_click_network.sh
```

这个脚本会把 `eth0` 的 `192.168.1.5/24` 写进 NetworkManager，并安装 udev 规则，让 USB-CAN 枚举成 `can*` 时自动配置 `1000000` bit/s 和 `restart-ms 100`。

设置桌面开机自启：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
./autostart/install_desktop_autostart.sh
./autostart/install_user_systemd_autostart.sh
```

推荐同时安装 user systemd 自启和桌面 XDG 自启。user systemd 是主要兜底入口，XDG 桌面自启保留为备用入口；两边都会调用 `autostart/boot_start.sh`。`boot_start.sh` 会记录当前 `boot_id`，同一次开机只允许真正启动一次，避免桌面自启重复触发导致两套雷达进程或两份互相干扰的日志。

自启诊断日志写到：

```text
logs/autostart/xdg_autostart.log
logs/autostart/boot_attempts.log
logs/autostart/boot_start_state.env
```

`boot_attempts.log` 只记录“自启入口有没有被调用”和跳过原因；`start_summary.txt`、`health_monitor.csv` 等仍然在每次实际运行的 `logs/autostart/run_<时间>_<原因>_<boot_id短码>_<pid>/` 目录里。目录名仍保留墙上时间方便人工查看，但真正用于区分强制重启的是 `boot_id短码 + pid`，避免开机初期系统时间未校准或连续强关重启时多个 run 落到同一秒而互相混日志。

由于香橙派开机后可能先用错误系统时间启动，随后被 NTP/chrony 校时，判断日志先后时不要只看文件名时间；优先看 `start_summary.txt` 里的 `boot_id`、`uptime_sec`、`elapsed_sec`，以及 `boot_attempts.log`。`uptime_sec` 是系统从本次开机以来经过的秒数，比墙上时间更可靠。

开机自启默认使用低负载重定位模式，并假设机器人从建图原点附近启动：

```text
RC_LIDAR_MODE=relocalization
RC_LIDAR_MAP_PATH=/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260624_190419.pcd
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1
RC_AUTOSTART_HEADLESS=1
RC_AUTOSTART_PLOTJUGGLER=0
RC_AUTOSTART_TF_COMPARE=auto
RC_AUTOSTART_LIDAR_XYZR=auto
RC_CAN_POSE_SOURCE=pose_topic
RC_CAN_POSE_TOPIC=/robot/pose_filtered
RC_HEALTH_MONITOR=1
```

也就是说，开机后会自动用 rc3.2 已验证的小场地图 `maps/mid360_map_20260624_190419.pcd` 做重定位，启动 TF、Livox、FAST-LIO、地图匹配、位姿接口、位姿滤波、CAN 和健康日志等核心链路；CAN 发 `/robot/pose_filtered`，但不会弹 RViz/PlotJuggler 界面。TF 对比发布节点和点云调试打印也会随 PlotJuggler/headless 策略默认跳过，避免香橙派 CPU 被调试工具抢走。需要临时开图形调试时，手动运行一键脚本并覆盖这些环境变量即可。

检查开机自启是否触发：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
tail -n 80 logs/autostart/boot_attempts.log
tail -n 120 logs/autostart/xdg_autostart.log
systemctl --user status rc-lidar-autostart.service
find logs/autostart -maxdepth 1 -type d -name 'run_*' -printf '%TY-%Tm-%Td %TH:%TM:%TS %f\n' | sort -r | head
```

如果强制断电重启后没有新的 `run_*_boot`，先看 `boot_attempts.log`：没有新记录说明自启入口没有被调用；有记录但没有 `run_*` 说明 `boot_start.sh` 被同 boot_id 保护跳过或 `start_all_terminals.sh` 失败。

临时配置示例：

```bash
ip -brief addr
sudo ip link set <雷达网口名> up
sudo ip addr add 192.168.1.5/24 dev <雷达网口名>
ping 192.168.1.124
```

如果提示地址已存在或网段冲突，先用 `ip addr show <雷达网口名>` 检查，再清理旧地址。

本机推荐配置：

```bash
sudo ip addr del 192.168.1.5/24 dev enx427ccccb233b 2>/dev/null || true
sudo ip link set eth0 up
sudo ip addr add 192.168.1.5/24 dev eth0 2>/dev/null || true
ip route get 192.168.1.124
arping -I eth0 -c 3 192.168.1.124
```

### CAN 接口

`rc_serial_driver` 默认使用：

```text
can_interface: can0
can_bitrate: 1000000
```

机器人业务 TF 坐标系和 CAN 输出坐标系统一为：

```text
x_tf = 右方
y_tf = 前方
z_tf = 上方
yaw_tf = 左转为正，0 表示初始化朝向
```

FAST-LIO / Livox 的原始 `camera_init`、`body`、`body_highrate` 帧仍保持雷达和算法原始坐标；机器人业务帧改为 `robot_origin`、`robot_base`、`robot_base_highrate`。其中 `map` 是 PCD 地图坐标，Y 正方向就是无头车的世界正方向；`map_localizer` 在重定位模式下发布 `map -> camera_init`。

发到 CAN 的业务坐标语义是：

```text
x_can = 右方
y_can = 前方
z_can = 上方
yaw_can = 左转为正，0 表示初始化朝向
```

普通 `tracking` 模式下，CAN 仍可直接使用机器人业务 TF：

```text
x_can = x_tf
y_can = y_tf
z_can = z_tf
yaw_can = yaw_tf
```

重定位 `relocalization` 模式下，CAN 默认严格使用滤波后的地图重定位机器人坐标：

```text
RC_CAN_POSE_SOURCE=pose_topic
RC_CAN_POSE_TOPIC=/robot/pose_filtered
```

也就是说，电控收到的是“机器人当前中心点相对于 `robot_origin` 的滤波后坐标”。如果 `/robot/pose_filtered` 没有刷新，CAN 会暂停发送，不会悄悄退回旧 TF 链路，避免坐标来源混乱。

CAN 帧格式保持不变：

```text
0x12: float x_can, float y_can
0x13: float z_can, float yaw_can
```

CAN 帧 ID 和 float 打包格式不变。需要临时回到 TF 链路时，可以手动设置 `RC_CAN_POSE_SOURCE=tf`，此时读取 `robot_origin -> robot_base_highrate`。

一键启动会先运行 `autostart/can_setup.sh`。如果没有指定 `CAN_INTERFACE`，它会自动寻找系统里第一个 `can*` 或 `slcan*` SocketCAN 接口；找到后会在免密 `sudo` 可用时自动配置：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

也可以手动运行：

```bash
./autostart/can_setup.sh --check-only
./autostart/can_setup.sh
```

如果日志里出现下面几类信息，说明不是 ROS 节点没有发，而是系统没有可用的 SocketCAN 设备：

```text
no SocketCAN interface was detected
requested CAN interface can0 does not exist
Failed to get interface index for 'can0'
CAN socket not ready
```

这时先重新插拔 USB-CAN，优先换稳定 USB 口或带供电 HUB，然后确认：

```bash
lsusb
ip -details link show type can
ip -brief link
```

期望能看到 `XCAN-USB` 或对应 USB-CAN 设备，以及 `can0`、`can1` 或 `slcan0` 这类接口。CAN 设备经常显示 `state UNKNOWN`，关键是 flags 里要有 `UP`，并且 `ip -details -statistics link show <CAN接口名>` 的 `TX packets` 在发送时持续增加。

每次运行 `./autostart/start_all_terminals.sh` 都会保存一份预检日志：

```text
logs/autostart/latest/can_preflight.txt
```

如果你想固定使用某个接口，例如 `can1`，可以临时指定：

```bash
CAN_INTERFACE=can1 ./autostart/can_setup.sh
CAN_INTERFACE=can1 ./autostart/start_all_terminals.sh
```

或者修改：

```text
src/rc_serial_driver/config/rc_serial_driver.yaml
```

把 `can_interface` 改为 `can1` 后重新编译或在 symlink install 下直接重新 source 环境。

## 推荐启动顺序

每个新终端都先执行：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
source /opt/ros/humble/setup.bash
source install/setup.bash
```

1. 启动静态 TF：

```bash
ros2 launch my_static_tf lidar_to_base.launch.py
ros2 launch my_static_tf map_to_init.launch.py
```

2. 启动 Livox 驱动：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

3. 启动 FAST-LIO：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
```

有桌面环境时可以打开 RViz：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=true
```

4. 启动 world 初始化节点：

```bash
ros2 run fastlio_world_init world_from_fastlio_init --ros-args \
  -p source_parent_frame:=camera_init \
  -p source_child_frame:=body \
  -p world_frame:=world \
  -p capture_duration:=3.0 \
  -p settle_time:=0.5 \
  -p sample_period:=0.02 \
  -p orientation_mode:=none
```

5. 启动 CAN 位姿发送：

```bash
ros2 launch rc_serial_driver serial_driver.launch.py
```

## 一键启动

当前 `autostart/` 脚本已经改成按脚本所在位置自动定位工作区，不再依赖旧的 `/home/rc/...` 路径。

先做一次不打开终端的预检查：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
RC_AUTOSTART_DRY_RUN=1 ./autostart/start_all_terminals.sh
```

确认输出里启动项都显示 `would launch` 后，再执行真正的一键启动：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
./autostart/start_all_terminals.sh
```

日志会写到：

```text
/home/orangepi/RC_LIDAR_NOW/logs/autostart/latest
```

一键启动默认会先执行一次旧进程清理，避免重复运行脚本后出现两套 Livox / FAST-LIO / EKF / CAN 发送节点同时发布。清理日志在：

```text
logs/autostart/latest/prestop.log
```

如果只是想手动停止当前雷达定位栈：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
./autostart/stop_all.sh
```

一般不要只靠关闭终端窗口来停止程序；低负载、开机自启、CAN 重启等 headless 模式会把进程放到后台，关闭终端不会可靠停止它们。确实需要跳过启动前清理时，可以临时设置：

```bash
RC_AUTOSTART_SKIP_PRESTOP=1 ./autostart/start_all_terminals.sh
```

默认会打开这些终端：

```text
TF_STATIC1       body -> robot_base
TF_STATIC2       map -> robot_origin
TF_STATIC3       body_highrate -> robot_base_highrate
TF_MAP_CAMERA_INIT
                 mapping/tracking 模式发布 map -> camera_init identity；relocalization 模式跳过，由 MAP_LOCALIZER 发布
LIVOX            Livox MID360 driver
FASTLIO          FAST-LIO mapping, RViz 默认不启动
HIGHRATE_BODY    robot_localization EKF；未安装时降级为 body -> body_highrate 静态 TF
TF_ROBOT_OFFSET  发布 robot_origin -> robot_base / robot_base_highrate 的 XYZ 对比 topic
TF_LIDAR_OFFSET  发布 map -> body 的 XYZ 对比 topic
CAN_SERIAL       rc_serial_driver
LIDAR_XYZR       点云调试打印
PLOTJUGGLER      打开 TF Compare 布局并订阅 /tf_compare topic
HEALTH_MONITOR   低频记录温度、CPU 负载、CPU 频率、地图质量和位姿稳定性
```

`HEALTH_MONITOR` 默认启动，开销很小，主要用于事后判断定位抖动是否和温度、CPU 负载、CPU 频率或地图匹配质量有关。需要关闭时设置：

```bash
RC_HEALTH_MONITOR=0 ./autostart/start_all_terminals.sh
```

默认运行模式是：

```text
RC_LIDAR_MODE=tracking
```

这个模式保持 FAST-LIO + `map -> camera_init` identity + 机器人业务 TF + CAN 输出链路，不依赖地图文件。新增的模式有：

```text
RC_LIDAR_MODE=mapping          # 建图，用 /map_save 保存 PCD 地图
RC_LIDAR_MODE=relocalization   # 读取 maps/mid360_map.pcd 做地图匹配重定位
```

当前默认策略：一键启动默认不打开 RViz，包括手动正常启动、低负载启动、建图和重定位。这样可以避免香橙派在现场运行时被 RViz 占用性能。需要临时打开 RViz 调试时，显式设置：

```bash
FASTLIO_RVIZ=true ./autostart/start_all_terminals.sh
```

建图或重定位时也一样：

```bash
FASTLIO_RVIZ=true RC_LIDAR_MODE=mapping ./autostart/start_all_terminals.sh
FASTLIO_RVIZ=true RC_LIDAR_MODE=relocalization ./autostart/start_all_terminals.sh
```

当前默认策略是：核心雷达/定位/CAN/健康日志节点都用 headless 后台方式启动，不再给每个节点单独弹终端窗口；手动启动时默认只额外打开 PlotJuggler，方便看 `/robot/pose`、`/robot/pose_filtered` 和 `/map_lidar/quality`。这样“正常启动”和“低负载启动”的核心链路几乎完全一致，差别主要是是否打开 PlotJuggler。

如果想要最低负载，关闭 PlotJuggler：

```bash
RC_AUTOSTART_HEADLESS=1 RC_AUTOSTART_PLOTJUGGLER=0 ./autostart/start_all_terminals.sh
```

PlotJuggler 默认在手动启动时打开，开机自启和 CAN 自动重启时默认不打开，避免反复弹 GUI 或占用香橙派 CPU。PlotJuggler 本身通常比 RViz、VS Code、浏览器轻很多，但订阅高频曲线和较长 buffer 时仍会消耗一些 CPU/内存；实车比赛/电控调车建议用低负载启动，调试观察时再打开 PlotJuggler。可以用下面的环境变量覆盖：

```bash
RC_AUTOSTART_PLOTJUGGLER=0 ./autostart/start_all_terminals.sh
RC_AUTOSTART_PLOTJUGGLER=1 RUN_REASON=boot ./autostart/start_all_terminals.sh
RC_AUTOSTART_PLOTJUGGLER=1 RUN_REASON=can_restart ./autostart/start_all_terminals.sh
```

TF 对比发布节点默认只在 PlotJuggler 会启动时自动启动；`LIDAR_XYZR` 点云调试打印默认只在显式非 headless 模式启动。需要恢复旧的“每个节点一个终端窗口”的重调试模式时：

```bash
RC_AUTOSTART_HEADLESS=0 ./autostart/start_all_terminals.sh
```

需要单独打开调试节点时：

```bash
RC_AUTOSTART_TF_COMPARE=1 RC_AUTOSTART_LIDAR_XYZR=1 \
  RC_AUTOSTART_HEADLESS=1 RC_AUTOSTART_PLOTJUGGLER=0 \
  ./autostart/start_all_terminals.sh
```

PlotJuggler 的 ROS2 插件会弹出确认窗口，本工程用 `xdotool` 自动按确认。如果没有安装：

```bash
sudo apt install -y xdotool
```

PlotJuggler 默认只负责自动打开 ROS2 streamer 和预选关键 topic，不再强制加载右侧曲线分屏布局。启动脚本会在打开 PlotJuggler 前同步用户配置：

```text
~/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini
[DataStreamROS2]
default_topics=...
```

这样 Topic Selector 里会默认选中 `/robot/pose`、`/robot/pose_filtered`、`/map_lidar/quality`、`/lidar/pose_filter/status` 和三个 `/tf_compare/...` topic。之前出现“只选中了 body/body_highrate 对比 topic”的原因，就是这个 `default_topics` 还保留着旧值；XML 里的 `selected_topics` 不一定会覆盖 ROS2 插件的用户配置。

PlotJuggler 启动前默认最多等待关键 topic `16s`，可用 `PLOTJUGGLER_WAIT_SECONDS` 覆盖。日志里如果看到 `Waiting for topics:` 后面列出一整组 topic，不一定代表这些 topic 永久不存在；启动早期也可能是 `ros2 topic list`/ROS graph 查询暂时失败。Rc 3.1 起脚本会把 `ros2 topic list failed` 单独写进日志，便于区分“topic 真的没发布”和“ROS graph 查询失败”。

如果确实想恢复旧的自动分屏布局，可以手动指定：

```bash
PLOTJUGGLER_TF_LAYOUT="$PWD/src/fastlio_world_init/config/tf_compare_plotjuggler.xml" \
  ./autostart/start_all_terminals.sh
```

### 健康日志：温度、负载与定位稳定性

每次一键启动都会默认启动轻量健康日志节点。它不订阅点云，只低频记录机器人位姿 topic、地图匹配质量和系统状态，目的是事后判断“抖动变大”是否和温度、CPU 负载、CPU 频率下降或地图匹配质量有关。

最近一次启动的日志路径是：

```text
logs/autostart/latest/health_monitor.csv
logs/autostart/latest/health_monitor_summary.json
logs/autostart/latest/health_monitor_run.json
logs/autostart/latest/health_monitor_stop.json
logs/autostart/latest/health_events.jsonl
```

CSV 默认每 0.2 秒记录一次，也就是 5Hz。它不订阅点云，正常情况下开销仍然很低，包含：

```text
max_temp_c
load1/load5/load15
cpu_freq_min_mhz/cpu_freq_max_mhz/cpu_freq_avg_mhz
raw_x/raw_y/raw_z/raw_yaw_deg
filtered_x/filtered_y/filtered_z/filtered_yaw_deg
raw_filtered_delta_xy_m
raw_window_xy_p2p_m
filtered_window_xy_p2p_m
motion_factor
is_stationary
stationary_filtered_window_xy_p2p_m
spike_candidate
sustained_motion
static_context_raw_xy_p2p_m
static_context_filtered_xy_p2p_m
map_quality_fitness
map_quality_jump_xy_m
map_quality_reason_code
map_quality_align_ms
map_quality_raw_scan_points
map_quality_scan_age_sec
map_quality_odom_age_sec
map_quality_output_jump_yaw_deg
map_quality_map_to_odom_delta_xy_m
map_quality_map_to_odom_delta_yaw_deg
map_quality_overlap_ratio
filter_status_accepted
filter_status_raw_step_xy_m
filter_status_speed_xy_mps
filter_status_alpha
filter_status_consecutive_rejects
filter_status_recovered
filter_status_motion_factor
filter_status_yaw_rate_degps
```

重点看三类数：

```text
motion_factor                         # 越接近 0 越像静止，越接近 1 越像真实运动
is_stationary                         # 1 表示当前窗口被认为适合统计静态漂移
stationary_filtered_window_xy_p2p_m    # 静止窗口内滤波位姿 XY 峰峰值
spike_candidate                       # 1 表示这一刻像单帧尖峰，不应直接当成正常运动
sustained_motion                      # 1 表示连续运动已超过阈值，静态漂移统计应暂停
static_context_raw_xy_p2p_m            # 单帧尖峰不会清空的原始静止上下文峰峰值
static_context_filtered_xy_p2p_m       # 单帧尖峰不会清空的滤波静止上下文峰峰值
map_quality_reason_code                # 0 accepted, 1 too_few_points, 2 not_converged, 3 bad_fitness
map_quality_align_ms                   # 地图匹配耗时；突然变大时优先怀疑负载/点云难匹配
map_quality_scan_age_sec/odom_age_sec  # scan/odom 延迟；突然变大时可能是调度或负载问题
filter_status_accepted/recovered       # 滤波器是否拒绝或从拒绝状态恢复
```

这样做是为了避免电控调车时把“机器人真的开走了”误算成静态漂移，同时又避免把静止时的单帧尖峰误判成持续运动后直接清空证据。`is_stationary` 仍表示即时静止判断；`static_context_*` 更适合排查“车没动但突然跳了几厘米”的情况。

`health_events.jsonl` 是低开销事件日志。它只在出现下面情况时写入一行 JSON，不会持续刷屏：

```text
raw_pose_step              # /robot/pose 单步跳变超过阈值
filtered_pose_step         # /robot/pose_filtered 单步跳变超过阈值
map_output_jump            # /map_lidar/quality 的 output_jump_xy 过大
map_bad_fitness            # fitness 超过诊断阈值
map_rejected               # 地图匹配本帧被拒绝
filter_rejected            # 滤波器本帧拒绝
filter_recovered           # 滤波器从连续拒绝中恢复
```

每个事件会保存触发原因、触发数值、事件前 3 秒和后 3 秒的温度、load、CPU 频率、地图质量、滤波状态和位姿窗口摘要。如果需要逐点上下文，再临时打开 `RC_HEALTH_MONITOR_EVENT_CONTEXT_CSV=1`，会额外写 `health_event_context.csv`；默认关闭是为了避免日志体积变大。

`summary.json` 会额外计算温度/负载和定位抖动的相关系数，例如：

```text
correlation.temp_vs_stationary_filtered_window_xy_p2p
correlation.load1_vs_stationary_filtered_window_xy_p2p
correlation.temp_vs_map_quality_jump_xy
```

如果温度或 `load1` 上升时，静止窗口峰峰值、`map_quality_jump_xy_m` 也同步上升，并且相关系数明显为正，就说明定位抖动很可能和热/负载相关。这个监测脚本本身很轻量；主要负载通常来自 PlotJuggler、RViz、VS Code、QQ、Clash、桌面环境和定位链路本身。

日志抗断电策略：

- `health_monitor.csv` 每行写入后都会 flush，默认每 5 秒额外 `fsync` 一次，尽量减少直接断电时丢失的数据。
- `health_monitor_summary.json` 使用临时文件写完后原子替换，避免断电时留下半截 JSON。
- `health_monitor_run.json` 在健康日志启动时立即写入，记录 `RUN_ID`、`RUN_REASON`、PID、topic、采样周期和输出路径。
- `health_monitor_stop.json` 只有健康日志正常收到停止信号并收尾时才会写入。CAN 重启通常会有 stop marker；直接断电通常没有 stop marker，可以据此判断上一轮是否异常掉电。
- `health_events.jsonl` 事件发生时立即 flush/fsync；它只记录异常窗口摘要，不记录点云。
- CAN 报文重启会生成新的 `run_<时间>_can_restart` 日志目录；直接断电再开机会生成新的 `run_<时间>_boot` 日志目录。
- 健康日志文件默认保留 7 天：`health_monitor.csv`、`health_monitor_summary.json`、`health_monitor_run.json`、`health_monitor_stop.json`、`health_events.jsonl`、`health_event_context.csv`。
- 原始运行日志目录也会做保守清理：`logs/autostart/run_*` 默认保留 30 天，并额外保留最近 10 次、`latest` 指向的当前运行、`latest_restart` 指向的最近重启日志。它不会删除 `maps/`、`logs/calibration/`、`report_assets/`，也不会按文件名去删 PCD 或校准报告。
- 每次一键启动会把原始日志清理结果写到当前运行目录的 `log_cleanup.log`。

可调参数：

```bash
RC_HEALTH_MONITOR=0                      # 关闭健康日志
RC_HEALTH_MONITOR_INTERVAL=0.2           # 采样周期，默认 0.2 秒，即 5Hz
RC_HEALTH_MONITOR_WINDOW=10.0            # 峰峰值窗口，默认 10 秒
RC_HEALTH_MONITOR_SUMMARY_INTERVAL=30.0  # summary 写入周期，默认 30 秒
RC_HEALTH_MONITOR_FSYNC_INTERVAL=5.0      # CSV fsync 周期，默认 5 秒；设为 0 只 flush 不定期 fsync
RC_HEALTH_MONITOR_RETENTION_DAYS=7       # 只清理旧健康日志文件；设为 0 可关闭自动清理
RC_AUTOSTART_LOG_RETENTION_DAYS=30       # 原始 run_* 日志目录保留天数；设为 0 可关闭原始日志清理
RC_AUTOSTART_LOG_KEEP_RECENT=10          # 即使超过天数，也额外保留最近 N 次 run_* 目录
RC_AUTOSTART_LOG_CLEANUP=1               # 设为 0 可关闭一键启动时的原始日志清理
RC_AUTOSTART_LOG_CLEANUP_DRY_RUN=1       # 只打印会删什么，不实际删除
RC_HEALTH_MONITOR_EVENT_PRE_SEC=3.0
RC_HEALTH_MONITOR_EVENT_POST_SEC=3.0
RC_HEALTH_MONITOR_SPIKE_RAW_STEP=0.02
RC_HEALTH_MONITOR_SPIKE_FILTERED_STEP=0.015
RC_HEALTH_MONITOR_SPIKE_MAP_JUMP=0.02
RC_HEALTH_MONITOR_SPIKE_FITNESS=0.02
RC_HEALTH_MONITOR_SUSTAINED_MOTION_SEC=2.0
RC_HEALTH_MONITOR_EVENT_CONTEXT_CSV=0     # 需要逐点事件上下文时设为 1
RC_HEALTH_MONITOR_STATIC_SPEED=0.015
RC_HEALTH_MONITOR_FULL_SPEED=0.12
RC_HEALTH_MONITOR_STATIC_YAW_RATE_DEG=5.0
RC_HEALTH_MONITOR_FULL_YAW_RATE_DEG=35.0
RC_MAP_QUALITY_OVERLAP=0                  # 默认关闭地图/scan 重合率诊断，打开会增加少量负载
```

如果要故意复现高负载发热问题，就不要使用 headless，也不要关闭 PlotJuggler；如果要做低负载对照实验，再加：

```bash
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
RC_AUTOSTART_TF_COMPARE=0 \
RC_AUTOSTART_LIDAR_XYZR=0 \
./autostart/start_all_terminals.sh
```

## 建图和重定位

新增建图/重定位链路参考 `Sa63R/rc_lidar_r2`，但保留当前工程的 TF 语义和经典 SocketCAN 报文格式。

目标链路是：

```text
Livox MID360 -> FAST-LIO -> /cloud_registered + /Odometry
建图模式：/map_save -> maps/mid360_map.pcd 或自动重命名后的 PCD
重定位观察链路：lidar_map_localization 发布 map -> camera_init；lidar_localization 查询 TF robot_origin -> robot_base_highrate，发布 /robot/pose -> pose_stability_filter -> /robot/pose_filtered
relocalization CAN 默认链路：/robot/pose_filtered -> rc_serial_driver
tracking CAN 默认链路：TF robot_origin -> robot_base_highrate/robot_base -> rc_serial_driver
```

高精度来自保存地图后的 NDT/ICP 地图匹配，不是单纯调 FAST-LIO 参数。`/robot/pose` 已经转换成机器人业务坐标：

```text
frame_id = robot_origin
x = 右方
y = 前方
z = 上方
yaw = 左转为正
```

`/lidar/pose` 是历史兼容名，内容和 `/robot/pose` 相同，实际表示的也是机器人中心点，不是雷达原点。重定位模式下 CAN 默认严格发送 `/robot/pose_filtered`；普通 FAST-LIO tracking 模式下可使用 `robot_origin -> robot_base_highrate` TF 链路。正负 0.5 cm 的静态误差目标需要依赖足够好的现场地图、稳定的静态环境、正确外参、雷达安装刚性和足够点云结构；代码提供重定位链路，但不能单独保证所有现场都达到这个误差。

### 建图

先启动建图模式：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=mapping ./autostart/start_all_terminals.sh
```

也可以用快捷脚本：

```bash
./autostart/start_mapping_terminals.sh
```

移动机器人覆盖需要重定位的场地，等 FAST-LIO 地图稳定后保存：

```bash
./autostart/save_fast_lio_map.sh
```

默认地图路径优先使用：

```text
maps/mid360_map.pcd
```

如果这个文件已经存在，并且没有手动设置 `RC_FASTLIO_MAP_SAVE_PATH`，`autostart/fastlio.sh` 会自动选择一个不冲突的新文件名，例如：

```text
maps/mid360_map_20260620_173012.pcd
```

此外，FAST-LIO 的 `/map_save` 服务在真正写文件前也会检查目标文件是否已经存在；如果存在，会再次自动改成带时间戳的新文件名。因此同一次运行里连续保存多次，也不会覆盖旧地图。实际本次 FAST-LIO 启动时使用的请求路径会写入：

```text
logs/autostart/latest/fastlio_map_save_path.txt
```

`autostart/save_fast_lio_map.sh` 会优先读取正在运行的 FAST-LIO 参数；如果 `/map_save` 自动重命名，脚本会从服务返回信息里打印真实保存路径。

`*.pcd` 默认不会提交到 GitHub，现场地图建议单独备份。

### 重定位

确认地图存在：

```bash
ls -lh maps/mid360_map.pcd
```

重要：当前 `lidar_map_localization` 是局部地图匹配，不是全局重定位。也就是说，它不能在完全不知道初始位置的情况下，从整张地图里自动找出机器人在哪里。之前出现过“机器人不是在建图原点启动，但输出仍接近 0”的现象，本质是把 `map -> camera_init` 初值默认假设成 identity，NDT 又持续匹配失败，容易造成误判。

大场地比赛时，不建议直接拿整张大场馆 PCD 做生产重定位。当前确认更稳的策略是：只绕比赛小场地建一张小 PCD 地图，不裁剪，机器人从固定上电点或接近建图原点的位置启动；如果不在原点启动，就必须给粗略初值。当前算法没有做 Scan Context、粒子全局搜索或多候选全局匹配，所以“拿全图自动找自己在哪里”不是它擅长的工作。

2026-06-24 实测可用检查点：使用只绕小场地建出的 `/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260624_190419.pcd`，不做裁剪，并用 `RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1` 从建图原点附近启动，重定位表现正确。这是当前优先复刻方案。

Rc 3.3 起，开机低功耗自启和 CAN 重启默认也使用这张小场地图，并保持 `RC_AUTOSTART_PLOTJUGGLER=0`。手动调试时如果需要 PlotJuggler，只额外把 `RC_AUTOSTART_PLOTJUGGLER=1` 打开即可。

现在默认不再假设当前启动点就是地图原点；重定位启动后，如果没有初始位姿，`MAP_LOCALIZER.log` 会提示等待 `/initialpose` 或 `RC_RELOCALIZATION_INITIAL_GUESS=1`，并且不会发布看起来像 0 的假定位结果。

如果机器人确实放在建图开始时的原点附近，可以显式允许地图原点初值：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
./autostart/start_all_terminals.sh
```

如果机器人不是从地图原点启动，需要给一个粗略初始位姿。下面的初值是 `map -> camera_init`，也就是 FAST-LIO 原始地图坐标系里的雷达/body 初始位姿，不是 `X右/Y前` 的业务坐标：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_INITIAL_GUESS=1 \
RC_RELOCALIZATION_INITIAL_X=0.0 \
RC_RELOCALIZATION_INITIAL_Y=0.0 \
RC_RELOCALIZATION_INITIAL_Z=0.0 \
RC_RELOCALIZATION_INITIAL_YAW=0.0 \
./autostart/start_all_terminals.sh
```

也可以运行后通过 RViz 或命令发布 `/initialpose` 来更新初始猜测。初值越接近真实位置，NDT/ICP 越容易收敛；如果 `MAP_LOCALIZER.log` 里持续出现 `map registration rejected` 且 `fitness` 远大于阈值，就表示重定位没有成功，不应该使用此时的 `/robot/pose`、`/lidar/pose` 或 CAN 坐标。

低负载重定位启动。这个模式不会降低雷达频率，只是不启动 PlotJuggler/RViz 等 GUI 调试负载：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_LIDAR_MAP_PATH="/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260624_190419.pcd" \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
./autostart/start_all_terminals.sh
```

开机和手动重定位默认会启动一个轻量就绪提示 `READY_NOTIFY`。它不会参与定位计算，只订阅 `/robot/pose_filtered` 和 `/map_lidar/quality`，当地图重定位连续稳定通过质量检查后，会写入：

```text
logs/autostart/<run>/relocalization_ready.json
```

如果当前有图形桌面，它还会弹出一个小终端窗口提示：

```text
RC LiDAR 重定位完成
现在可以移动小车
```

这个时间表示“重定位输出已经连续稳定”的时刻，不是进程刚启动的时刻。当前默认判据是：`/robot/pose_filtered` 刷新正常，`/map_lidar/quality` 连续 accepted/converged，`fitness <= 0.05`，并且稳定 3 秒。一般从一键启动开始到 ready 预计约几十秒；冷启动、网口慢、地图匹配慢或现场点云质量差时会更久。实际耗时以 `relocalization_ready.json` 里的 `elapsed_sec` 为准。

可调参数：

```bash
RC_READY_NOTIFY=0                    # 关闭就绪提示
RC_READY_NOTIFY_STABLE_SEC=3.0       # 连续稳定多久才认为 ready
RC_READY_NOTIFY_TIMEOUT_SEC=120.0    # 超时时间
RC_READY_NOTIFY_MAX_FITNESS=0.05     # ready 的 fitness 上限
RC_READY_NOTIFY_WINDOW_SECONDS=20    # 提示窗口自动关闭时间
```

CAN 报文触发的自动重启默认不弹 ready 窗口，避免电控反复重启时刷屏；如确实需要，重启环境里设置 `RC_READY_NOTIFY=1`。

如果想打开图形窗口调试，默认会打开 PlotJuggler，但不会打开 RViz，也不会给每个节点单独弹终端窗口：

```bash
RC_LIDAR_MODE=relocalization ./autostart/start_all_terminals.sh
```

如果想完全贴近开机/比赛低负载状态，只是不看 PlotJuggler：

```bash
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
./autostart/start_all_terminals.sh
```

如果重定位调试时也需要 RViz：

```bash
FASTLIO_RVIZ=true RC_LIDAR_MODE=relocalization ./autostart/start_all_terminals.sh
```

重定位模式会新增：

```text
MAP_LOCALIZER   lidar_map_localization，发布 /map_lidar/odom
POSE_INTERFACE lidar_localization，从 TF robot_origin -> robot_base_highrate 发布 /robot/pose，同时保留兼容名 /lidar/pose
POSE_FILTER    lidar_localization，发布 /robot/pose_filtered，同时保留兼容名 /lidar/pose_filtered
READY_NOTIFY   等待 /robot/pose_filtered 与 /map_lidar/quality 稳定后弹出就绪提示
```

验证 topic：

```bash
ros2 topic echo /map_lidar/odom --once
ros2 topic echo /map_lidar/quality --once
ros2 topic echo /robot/pose --once
ros2 topic echo /robot/pose_filtered --once
ros2 topic echo /lidar/pose --once
ros2 topic echo /lidar/pose_filtered --once
ros2 topic echo /lidar/pose_filter/status --once
ros2 topic echo /lidar/point --once
```

重定位模式下，CAN 默认严格使用滤波后的地图重定位 topic：

```text
pose_source=pose_topic
pose_topic=/robot/pose_filtered
```

这条链路经过 `map -> camera_init` 重定位 TF 和 `robot_origin -> robot_base_highrate` 查询，得到 `/robot/pose -> /robot/pose_filtered`。电控收到的是地图重定位后的滤波机器人中心点坐标。如果 `/robot/pose_filtered` 没有刷新或 topic 不存在，CAN 会暂停发送，不会自动退回 TF。

Rc 3.0 起，重定位模式下 CAN 还会默认等待 `/map_lidar/quality` 连续稳定后才发送。判据和 READY 提示保持同一方向：本帧 accepted/converged、`fitness <= 0.05`、连续 accepted 数达到阈值，并且质量 topic 没有超时。这样即使 `/robot/pose_filtered` 已经存在，只要地图匹配还没真正稳定，电控也不会收到早期的假定位坐标。

可调参数：

```bash
RC_CAN_REQUIRE_READY=1                 # relocalization 模式默认开启；tracking/mapping 默认关闭
RC_CAN_READY_QUALITY_TOPIC=/map_lidar/quality
RC_CAN_READY_TIMEOUT_SEC=1.0
RC_CAN_READY_MAX_FITNESS=0.05
RC_CAN_READY_MIN_ACCEPTS=5
RC_CAN_READY_REQUIRE_ACCEPTED=1
RC_CAN_READY_REQUIRE_CONVERGED=1
```

如果需要临时回退旧 TF 链路，必须手动覆盖：

```bash
RC_CAN_POSE_SOURCE=tf ./autostart/start_all_terminals.sh
```

`/robot/pose_filtered` 是低延迟自适应滤波后的机器人中心点位姿，主要用于观察是否能降低地图重定位输出的毫米到厘米级抖动，以及拒绝少数硬离群点。它不会让原始点云匹配本身变得更准；判断原始定位质量仍要看 `/robot/pose`、`/map_lidar/quality` 和现场地图质量。

Rc 2.9 起，滤波核心参考 r2 仓库的 normal/beta 方案：

- normal 仍是默认生产链路，输出 `/robot/pose_filtered`，CAN 默认继续发这个 topic。
- beta 是尖峰确认滤波，输出 `/robot/pose_filtered_beta` 和 `/lidar/pose_filter_beta/status`。它在静止时遇到疑似尖峰会先 hold，若后续回落则拒绝，若连续同方向运动达到确认次数才释放为真实运动。
- beta 默认不启动，目的是保护低负载开机和 CAN 调车链路；需要观察时手动打开。

打开 beta 对比，但 CAN 仍发 normal：

```bash
RC_POSE_FILTER_BETA=1 ./autostart/start_all_terminals.sh
```

确认 beta 更适合现场后，才显式让 CAN 发 beta：

```bash
RC_POSE_FILTER_BETA=1 \
RC_POSE_FILTER_SELECTED=beta \
./autostart/start_all_terminals.sh
```

普通 tracking 模式默认仍是：

```text
pose_source=tf
```

如果需要手动覆盖：

```bash
RC_CAN_POSE_SOURCE=tf ./autostart/start_all_terminals.sh
RC_CAN_POSE_SOURCE=pose_topic RC_CAN_POSE_TOPIC=/robot/pose_filtered ./autostart/start_all_terminals.sh
```

PlotJuggler 里看机器人中心点滤波前后抖动，优先选这些曲线：

```text
/robot/pose/pose/position/x
/robot/pose_filtered/pose/position/x
/robot/pose_filtered_beta/pose/position/x
/robot/pose/pose/position/y
/robot/pose_filtered/pose/position/y
/robot/pose_filtered_beta/pose/position/y
/robot/pose/pose/position/z
/robot/pose_filtered/pose/position/z
/robot/pose_filtered_beta/pose/position/z
```

滤波观察要区分几件事：

- `/robot/pose` 是滤波前的地图重定位观察值，来源仍然是 `/map_lidar/odom -> pose_from_odom`。Rc 2.4 没有修改 `/robot/pose` 的生成算法、地图匹配参数或坐标转换逻辑。如果重启后看到 `/robot/pose` 比之前稳定，通常不是代码直接调稳了它，而是这次运行的状态更干净、地图匹配收敛更好、现场动态干扰更少，或者 PlotJuggler 里选到的曲线和之前不同。
- `/robot/pose_filtered` 是默认滤波后的观察值。滤波目标是静止时压住小抖动，运动时尽量保留连续轨迹；绕点转圈时，连续的正弦/余弦曲线通常比台阶或锯齿更符合真实运动。
- `/robot/pose_filtered_beta` 只有 `RC_POSE_FILTER_BETA=1` 时发布，主要用于和 normal 对比静止尖峰抑制效果；PlotJuggler 也只会在 beta 打开时自动等待/订阅 beta topic。
- 当前滤波用 `xy_speed` 和 `yaw_rate` 计算运动因子：静止时使用较强平滑，明显运动时快速跟随原始曲线，中间状态平滑过渡。
- 硬离群点仍会被短暂拒绝并保持旧输出；但如果连续拒绝超过默认 10 次，或持续超过默认 1 秒，会认为这已经不是单帧异常，而是位姿发生了真实变化，于是重新捕获当前位姿，避免输出永久停成水平线。
- 当前生产策略是：重定位模式下 CAN 默认使用 `/robot/pose_filtered`，tracking 模式下才继续使用 TF 的 `robot_origin -> robot_base_highrate/robot_base`。因此调车时如果启用 `RC_LIDAR_MODE=relocalization`，滤波表现会直接影响电控收到的坐标。

滤波默认参数可以通过环境变量覆盖：

```bash
RC_POSE_FILTER_STATIC_ALPHA=0.12
RC_POSE_FILTER_MOVING_ALPHA=0.92
RC_POSE_FILTER_STATIC_SPEED=0.03
RC_POSE_FILTER_FULL_SPEED=0.20
RC_POSE_FILTER_STATIC_YAW_RATE_DEG=8.0
RC_POSE_FILTER_FULL_YAW_RATE_DEG=45.0
RC_POSE_FILTER_MAX_XY_JUMP=0.50
RC_POSE_FILTER_MAX_XY_SPEED=5.0
RC_POSE_FILTER_MAX_YAW_JUMP_DEG=90.0
RC_POSE_FILTER_SPIKE_STEP=0.05
RC_POSE_FILTER_BETA_SPIKE_STEP=0.012
RC_POSE_FILTER_BETA_CONFIRM_SAMPLES=3
RC_POSE_FILTER_BETA_RETURN_TOLERANCE=0.006
```

`/lidar/pose_filter/status` 使用 `std_msgs/Float32MultiArray`，字段含义：

```text
data[0]  accepted，1 表示本帧被接受，0 表示被判定为突跳并沿用上一帧滤波值
data[1]  raw_x
data[2]  raw_y
data[3]  filtered_x
data[4]  filtered_y
data[5]  raw_xy_step
data[6]  raw_xy_speed
data[7]  filter_alpha
data[8]  raw_yaw
data[9]  filtered_yaw
data[10] reject_count
data[11] accept_count
data[12] consecutive_rejects
data[13] recovered，1 表示连续拒绝后重新捕获当前位姿，避免滤波输出永久卡死
data[14] motion_factor，0 表示按静止强滤波，1 表示按运动快速跟随
data[15] yaw_rate_deg，当前原始 yaw 变化速率，单位 deg/s
data[16] filter_mode_code，0=normal，1=beta
data[17] reason_code，0=initial，1=accepted，2=stationary_spike，3=hard_reject，4=recovered，5=beta_hold，6=beta_spike_rejected，7=beta_confirmed_motion
data[18] hold_state_code，0=idle，1=holding，2=rejected，3=released
data[19] confirm_count，beta 连续确认计数
data[20] spike_candidate，1 表示本帧被视为疑似尖峰或硬离群
```

`/map_lidar/quality` 使用 `std_msgs/Float32MultiArray`，字段含义：

```text
data[0] accepted
data[1] converged
data[2] fitness
data[3] fitness_threshold
data[4] consecutive_rejects
data[5] filtered_scan_points
data[6] filtered_map_points
data[7] output_jump_xy
data[8] has_initial_pose
data[9] consecutive_accepts
data[10] reason_code，0 accepted, 1 too_few_points, 2 not_converged, 3 bad_fitness
data[11] align_ms，地图匹配耗时
data[12] raw_scan_points，降采样前 scan 点数
data[13] scan_age_sec，当前处理时 scan 的年龄
data[14] odom_age_sec，当前处理时 odom 的年龄
data[15] output_jump_yaw_deg，输出位姿 yaw 单步跳变量
data[16] map_to_odom_delta_xy_m，本轮地图匹配对 map->odom 的 XY 修正量
data[17] map_to_odom_delta_yaw_deg，本轮地图匹配对 map->odom 的 yaw 修正量
data[18] overlap_ratio，默认 -1；RC_MAP_QUALITY_OVERLAP=1 时低频计算 scan/map 近邻重合率
```

### 地图路径

重定位默认读取：

```text
maps/mid360_map.pcd
```

如果建图时自动保存成了带时间戳的新文件，重定位时需要指定那份地图：

```bash
RC_LIDAR_MAP_PATH="/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260620_173012.pcd" \
RC_LIDAR_MODE=relocalization \
./autostart/start_all_terminals.sh
```

临时使用其他地图：

```bash
RC_LIDAR_MAP_PATH=/home/orangepi/maps/site_a.pcd \
RC_LIDAR_MODE=relocalization \
./autostart/start_all_terminals.sh
```

### 局部地图准备

如果在大场地里只比赛一个小区域，首选方案是直接只绕比赛小场地建一张小 PCD 地图，不裁剪。Rc 3.2 的可用检查点就是这个方案：`maps/mid360_map_20260624_190419.pcd`，配合 `RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1` 从建图原点附近启动。

`pcd_prepare_local_map` 裁剪工具仍保留，但这次可用方案没有用到它。它只作为备用：当手里只有一整张大场馆 PCD，来不及重新绕小场地建图时，再尝试从大图裁出比赛区域。迁移到 R2 时优先复刻“直接绕小场建小图”的流程，不要先在裁剪参数上浪费时间。

新增工具不会修改原始 PCD；如果输出路径已经存在，会自动添加时间戳保存，避免覆盖旧地图。

矩形裁剪示例，坐标单位是原始 PCD/FAST-LIO map 坐标系里的米：

```bash
cd /home/orangepi/RC_LIDAR_NOW
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

圆形裁剪示例：

```bash
ros2 run lidar_map_localization pcd_prepare_local_map \
  --input maps/mid360_map.pcd \
  --output maps/competition_local.pcd \
  --leaf 0.10 \
  --center-x 0.0 --center-y 0.0 --radius 6.0 \
  --margin 2.0
```

工具会同时生成：

```text
maps/competition_local.pcd.report.json
```

报告里记录输入/输出路径、裁剪范围、原始点数、有效点数、裁剪后点数和降采样后点数。生产启动时指定这份局部图：

```bash
cd /home/orangepi/RC_LIDAR_NOW
RC_LIDAR_MODE=relocalization \
RC_LIDAR_MAP_PATH=/home/orangepi/RC_LIDAR_NOW/maps/competition_local.pcd \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
./autostart/start_all_terminals.sh
```

如果你明确想保存到某个固定路径，可以手动指定 FAST-LIO 保存路径：

```bash
RC_FASTLIO_MAP_SAVE_PATH=/home/orangepi/maps/site_a.pcd \
RC_LIDAR_MODE=mapping \
./autostart/start_all_terminals.sh
```

显式设置 `RC_FASTLIO_MAP_SAVE_PATH` 时脚本会尊重该请求路径；如果该文件已存在，`/map_save` 仍会自动生成不冲突的新文件名。

### 地图质量评分

可以离线检查 PCD 地图的基本质量：

```bash
cd /home/orangepi/RC_LIDAR_NOW
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run lidar_map_localization pcd_quality_report \
  --map maps/mid360_map.pcd
```

报告会输出点数、XYZ 范围、Z 高度分布、不同体素大小下的占用数量和点密度。这个工具不会自动修改地图，只用于比较多次建图结果。一般来说，地图点数过低、10cm XY 网格点密度过低、Z 方向结构太弱、或者出现很长的漂移尾巴，都会让重定位更容易抖动。

## 自转外参校准

如果机械给出的雷达相对底盘中心坐标存在微小偏差，机器人原地自转时，`robot_origin -> robot_base_highrate` 的 XY 会出现本不该有的小圆波动。可以用自转校准工具记录这段数据并自动计算建议修正值。

先正常启动低负载定位栈，等待重定位 ready 或确认 `robot_origin -> robot_base_highrate` 可查询，然后运行：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fastlio_world_init rotation_extrinsic_calibrator \
  --duration 40 \
  --rate 50
```

运行后让机器人尽量在原地慢速自转，建议至少转满一圈，最好转 1.5 到 2 圈。转速不需要恒定；脚本会按 yaw 角度分箱平均，避免某个角度停留太久导致拟合被过度加权。脚本到达 `--duration` 后会自动结束；也可以按 `Ctrl-C` 提前结束，脚本会写出已采集的部分数据。如果需要连同雷达定位栈一起停止，也可以运行：

```bash
./autostart/stop_all.sh
```

脚本默认记录：

```text
robot_origin -> body_highrate
robot_origin -> robot_base_highrate
body_highrate -> robot_base_highrate
```

输出目录默认是：

```text
logs/calibration/rotation_extrinsic_<时间>/
```

里面会生成：

```text
samples.csv
summary.json
report.md
```

其中 `summary.json` 和 `report.md` 会给出：

```text
sample_count
fit_sample_count
yaw_span_deg
yaw_sources
yaw_bin_deg
xy_peak_to_peak
fitted_error_xy
current_static_xy
suggested_static_xy
residual_before_after
```

当前已经应用过一次自转校准结果：

```text
数据目录：logs/calibration/rotation_extrinsic_20260619_175422/
记录时长：37.0 s
yaw 覆盖：约 625 deg
拟合前 RMS：0.035770 m
拟合后 RMS：0.022923 m
```

根据该记录，外参从：

```text
x=-0.240, y=0.315, z=-0.760, yaw=pi
```

调整为：

```text
x=-0.240, y=0.287302, z=-0.760, yaw=pi/2
```

其中 `x` 的拟合修正量约 `0.01 mm`，低于实际安装和定位噪声，因此保持 `-0.240` 不变；`y` 按建议值从 `0.315` 修正为 `0.287302`。之后为了满足电控坐标约定，机器人业务帧 yaw 从 `pi` 调整为 `pi/2`，使 `X=右方`、`Y=前方`。Rc 3.4 起这组三处静态 TF 必须保持一致：

```text
map -> robot_origin
body -> robot_base
body_highrate -> robot_base_highrate
```

默认只输出建议值，不会自动修改 TF。确认报告可信后，把 `summary.json` 和 `samples.csv` 发出来，再同步修改三处静态 TF：

```text
src/my_static_tf/launch/map_to_init.launch.py
src/my_static_tf/launch/lidar_to_base.launch.py
src/my_static_tf/launch/lidar_to_base_highrate.launch.py
```

本校准只修正 XY；`z`、`roll`、`pitch`、`yaw=pi/2` 保持不变。

## 运行验证

FAST-LIO 无雷达数据时也应能启动并等待 topic。本次短启动验证看到：

```text
[laser_mapping]: p_pre->lidar_type 1
Multi thread started
[laser_mapping]: Node init finished.
```

检查 topic：

```bash
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /Odometry --once
ros2 topic echo /tf_compare/robot_origin_to_robot_base --once
ros2 topic echo /tf_compare/robot_origin_to_robot_base_highrate --once
ros2 topic echo /tf_compare/map_to_body --once
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo camera_init body
ros2 run tf2_ros tf2_echo map robot_origin
ros2 run tf2_ros tf2_echo body robot_base
ros2 run tf2_ros tf2_echo body_highrate robot_base_highrate
ros2 run tf2_ros tf2_echo robot_origin robot_base
ros2 run tf2_ros tf2_echo robot_origin robot_base_highrate
```

## 本次修改

本次为了让当前环境能够稳定编译和运行，做了以下小修改，没有改变核心算法逻辑。

1. `src/fastlio_world_init/fastlio_world_init/world_from_fastlio_init.py`
   - 修复 Ctrl-C 或超时退出时重复调用 `rclpy.shutdown()` 导致的异常。
   - 把日志里的静态 TF 发布方向改成实际方向：`source_parent_frame -> world_frame`。

2. `src/rc_serial_driver/launch/serial_driver.launch.py`
   - 原 launch 文件加载 `config/serial_driver.yaml`，但实际配置文件名是 `rc_serial_driver.yaml`。
   - 已改为加载 `config/rc_serial_driver.yaml`，否则节点会启动但不会读取配置文件。
   - 增加 `RC_CAN_POSE_SOURCE` 和 `RC_CAN_POSE_TOPIC` 环境变量支持；重定位模式默认严格读 `/robot/pose_filtered`，tracking 模式默认读 TF。

3. `autostart/*.sh`
   - 统一通过 `autostart/common_env.sh` 加载当前工作区环境。
   - 去掉旧路径 `/home/rc/...`、`/home/rc2/...`、`~/jwt/...`。
   - `start_all_terminals.sh` 改为自动定位当前 `autostart/`，日志写入本工作区的 `logs/autostart/`。
   - `serial_driver.sh` 会尝试免密 sudo 拉起 CAN；如果没有权限，会打印手动命令并继续启动节点。
   - `highrate_body.sh` 在缺少 `robot_localization` 时发布 `body -> body_highrate` 静态 TF 作为降级兜底。

4. `src/my_static_tf/launch/lidar_to_base_highrate.launch.py`
   - 新增高频 base_link 静态 TF：`body_highrate -> base_link_highrate`，用于修复原脚本引用了不存在 launch 文件的问题。

4a. Rc 3.4 无头车 TF 树重写
   - 生产 TF 从旧 `world -> init`、`body -> base_link`、`body_highrate -> base_link_highrate` 改为 `map -> robot_origin`、`body -> robot_base`、`body_highrate -> robot_base_highrate`。
   - `map` 表示 PCD 地图坐标，Y 正方向作为无头车世界正方向；重定位模式由 `map_localizer` 发布 `map -> camera_init`，mapping/tracking 模式发布 identity `map -> camera_init`。
   - `/robot/pose` 不再由 `/map_lidar/odom` 手算相对位姿，而是从 TF 查询 `robot_origin -> robot_base_highrate`；滤波和 CAN 继续使用 `/robot/pose_filtered`。

5. `src/my_static_tf/launch/map_to_init.launch.py`
   - 历史版本曾把静态 TF 从 `lidar -> init` 调整为 `world -> init`；Rc 3.4 后生产树改为 `map -> robot_origin`。

6. `src/my_static_tf/launch/*.launch.py`
   - 按雷达安装位置更新机器人中心相对雷达坐标系的静态外参。
   - 根据 `logs/calibration/rotation_extrinsic_20260619_175422/` 的自转校准结果修正 Y 方向外参。
   - 将机器人业务帧改为 `X右/Y前/Z上`。
   - 当前统一参数为：`x=-0.240, y=0.287302, z=-0.760, roll=0, pitch=0, yaw=pi/2`。
   - Rc 3.4 后 `map -> robot_origin`、`body -> robot_base`、`body_highrate -> robot_base_highrate` 三套 TF 使用同一组外参。

7. `autostart/world_tf.sh`
   - 这是旧 tracking 链路的初始化工具；Rc 3.4 后生产启动默认跳过，改用 `map` 和 `robot_origin` 业务树。

8. `src/fastlio_world_init/fastlio_world_init/robot_tf_offsets.py`
   - TF 对比 topic 更新为 `/tf_compare/robot_origin_to_robot_base` 和 `/tf_compare/robot_origin_to_robot_base_highrate`。
   - 分别发布 `robot_origin -> robot_base` 与 `robot_origin -> robot_base_highrate` 的 XYZ 平移。

9. `src/fastlio_world_init/fastlio_world_init/lidar_tf_offset.py`
   - TF 对比 topic 更新为 `/tf_compare/map_to_body`。
   - 发布 `map -> body` 的 XYZ 平移，保持雷达原始坐标轴。

10. `src/fastlio_world_init/config/tf_compare_plotjuggler.xml`
    - 保留 PlotJuggler ROS2 streamer 配置模板，包含 `/robot/pose`、`/robot/pose_filtered`、`/map_lidar/quality`、`/lidar/pose_filter/status` 和三个 `/tf_compare/...` topic。
    - 默认启动不再强制加载自动分屏曲线布局；需要时可用 `PLOTJUGGLER_TF_LAYOUT=...` 手动指定。

11. `autostart/robot_tf_offsets.sh`、`autostart/lidar_tf_offset.sh`、`autostart/plotjuggler_tf_compare.sh`
    - 一键启动时自动启动 TF 对比发布节点和 PlotJuggler。
    - `plotjuggler_tf_compare.sh` 在重定位模式下会等待 `/robot/pose`、`/robot/pose_filtered`、质量 topic 和 `/tf_compare/...` topic 出现，写入 PlotJuggler 的 `DataStreamROS2/default_topics`，再打开 PlotJuggler。

12. `src/fastlio_world_init/fastlio_world_init/rotation_extrinsic_calibrator.py`
    - 新增自转外参校准工具。
    - 原地自转时记录 `robot_origin -> body_highrate`、`robot_origin -> robot_base_highrate`、`body_highrate -> robot_base_highrate`。
    - 额外记录 `camera_init -> body`、`map -> body` 和 `/Odometry` 的 yaw 诊断，避免只看单一路径时误判旋转圈数。
    - 自动拟合 XY 小圆偏心，输出 `samples.csv`、`summary.json`、`report.md`，只给出建议值，不自动改 TF 文件。

13. `autostart/livox.sh`、`autostart/setup_livox_net.sh`
   - `livox.sh` 会在启动 Livox 驱动前自动调用 `setup_livox_net.sh`。
   - `setup_livox_net.sh` 用于把接雷达的网口配置为 `192.168.1.5/24` 并用 ARP 探测雷达。
   - 可用 `RC_AUTOSTART_SETUP_LIVOX_NET=0` 跳过自动配置。

14. `src/rc_serial_driver/config/rc_serial_driver.yaml`
   - 增加 `restart_script`，指向当前工作区的 `autostart/restart_lidar_stack.sh`。

15. `autostart/can_setup.sh`、`autostart/serial_driver.sh`、`autostart/start_all_terminals.sh`
   - 新增 CAN 预检和恢复脚本：自动检测 `can*` / `slcan*`，存在时配置 `1000000` bit/s 和 `restart-ms 100`。
   - `start_all_terminals.sh` 会写入 `logs/autostart/latest/can_preflight.txt`。
   - 如果 USB-CAN 没有枚举出任何 CAN 接口，`CAN_SERIAL` 会停止启动并在日志里提示重插、换 USB 口或带供电 HUB。
   - `rc_serial_driver` 发给电控的 CAN 坐标为 `X右/Y前/Z上`。tracking 模式默认使用 TF，relocalization 模式默认使用 `/robot/pose_filtered`，不再悄悄 fallback 到旧 TF 链路。

16. `autostart/install_one_click_network.sh`
   - 一次性配置免密码一键启动所需的网络环境。
   - 持久化 `eth0 -> 192.168.1.5/24`。
   - 安装 udev 规则，USB-CAN 枚举为 `can*` 时自动拉起 CAN 口。

17. `autostart/boot_start.sh`、`autostart/rc_lidar_autostart.desktop`、`autostart/install_desktop_autostart.sh`、`autostart/rc-lidar-autostart.service`、`autostart/install_user_systemd_autostart.sh`
   - 新增 user systemd 自启入口，并保留图形桌面 XDG 自启入口作为备用。
   - 两个入口都调用 `boot_start.sh`，并通过 `boot_id` 状态文件保证同一次开机只真正启动一次。
   - 自启入口诊断写到 `logs/autostart/xdg_autostart.log`、`logs/autostart/boot_attempts.log`、`logs/autostart/boot_start_state.env`。
   - `start_all_terminals.sh` 会在创建 `start_summary.txt`、预检日志和最终启动摘要后主动 `sync`，降低强制断电时日志完全丢失的概率。

18. `autostart/stop_all.sh`、`autostart/start_all_terminals.sh`
   - 新增统一停止脚本，用于清理本项目启动的 Livox、FAST-LIO、静态 TF、EKF、CAN 发送、RViz、PlotJuggler、自转校准等进程。
   - `start_all_terminals.sh` 默认会在真正启动前调用 `stop_all.sh --quiet`，防止重复启动导致多个 `/odometry/filtered` publisher 或重复 TF。
   - `RC_AUTOSTART_DRY_RUN=1` 时不会执行清理；需要临时跳过可设置 `RC_AUTOSTART_SKIP_PRESTOP=1`。

19. `src/lidar_map_localization`
   - 新增 PCD 地图重定位节点 `map_localizer`。
   - 默认读取 `maps/mid360_map.pcd`，订阅 `/cloud_registered` 和 `/Odometry`，输出 `/map_lidar/odom`、`/map_lidar/pose`、`/localization/map`、`/map_lidar/quality`。
   - 当前帧名适配为 FAST-LIO 原始帧：`map -> camera_init -> body`。
   - 新增 `pcd_quality_report`，用于离线检查 PCD 地图点数、范围和体素密度。
   - Rc 3.0 新增 `pcd_prepare_local_map`，用于从大场馆 PCD 裁出比赛局部图并降采样；输出已存在时自动改名，避免覆盖原图或旧局部图。
   - Rc 3.2 确认可用方案是“直接绕比赛小场地建小 PCD，不裁剪”。本机可用地图为 `maps/mid360_map_20260624_190419.pcd`，裁剪工具保留为备用但本检查点没有用到。

20. `src/lidar_localization`
   - 新增 `pose_from_odom` 位姿接口。
   - 建图模式下可把 `/Odometry` 转成相对启动点的 `/robot/pose`，同时保留旧兼容名 `/lidar/pose`。
   - Rc 3.4 后重定位模式下从 TF 查询 `robot_origin -> robot_base_highrate` 生成 `/robot/pose`，坐标语义为 `X右/Y前/Z上`。
   - 新增 `pose_stability_filter`，将 `/robot/pose` 平滑为 `/robot/pose_filtered`，同时保留旧兼容名 `/lidar/pose_filtered`；`RC_POSE_FILTER=0` 可关闭。滤波器按 `xy_speed` 和 `yaw_rate` 自适应调整强度，静止时压抖动，运动时快速跟随，并带连续拒绝后的恢复机制，避免机器人移动后滤波输出永久停在旧值。
   - Rc 2.9 参考 r2 的 `pose_filter.py` 增加 `filter_mode=normal|beta`。normal 仍是默认生产链路；beta 输出 `/robot/pose_filtered_beta` 和 `/lidar/pose_filter_beta/status`，通过短时 hold/确认抑制静止尖峰。beta 默认不启动，需要 `RC_POSE_FILTER_BETA=1` 手动开启对比。

21. `autostart/start_mapping_terminals.sh`、`autostart/save_fast_lio_map.sh`、`autostart/map_localizer.sh`、`autostart/pose_interface_mapping.sh`、`autostart/pose_interface_relocalization.sh`、`autostart/pose_stability_filter.sh`
   - 新增建图保存和地图重定位的一键脚本。
   - `RC_LIDAR_MODE=mapping` 用于建图保存；`RC_LIDAR_MODE=relocalization` 用于基于地图的高精度重定位。
   - 未设置 `RC_FASTLIO_MAP_SAVE_PATH` 时，若默认 `maps/mid360_map.pcd` 已存在，会自动保存为带时间戳的新 PCD，避免覆盖旧地图。
   - 重定位模式默认启动 normal 位姿滤波，并默认把 `/robot/pose_filtered` 作为 CAN 发送来源；`RC_POSE_FILTER_BETA=1` 可额外启动 beta 对比，`RC_POSE_FILTER_SELECTED=beta` 才会把 CAN 来源切到 `/robot/pose_filtered_beta`。`stop_all.sh` 和 CAN 重启脚本已同步清理 normal/beta 滤波节点，避免模式切换后残留旧进程。
   - Rc 3.0 起，重定位模式下 CAN 默认启用 `RC_CAN_REQUIRE_READY=1`，等待 `/map_lidar/quality` 连续稳定后才发送 `/robot/pose_filtered`，避免开机早期或匹配失败时把假坐标发给电控。

22. `autostart/thermal_pose_monitor.py`、`autostart/thermal_pose_monitor.sh`
   - 新增默认启动的轻量健康日志；`RC_HEALTH_MONITOR=0` 可关闭，旧变量 `RC_THERMAL_POSE_MONITOR` 仍兼容。
   - 默认 0.2 秒记录一次，也就是 5Hz，记录温度、`load1/load5/load15`、CPU 频率、`/robot/pose`、`/robot/pose_filtered`、可选 `/robot/pose_filtered_beta`、`/map_lidar/quality`、normal/beta 滤波状态、运动状态和静止窗口 XY 峰峰值。
   - 输出 `logs/autostart/latest/health_monitor.csv`、`health_monitor_summary.json`、`health_monitor_run.json`、`health_monitor_stop.json` 与 `health_events.jsonl`，用于判断定位抖动是否和热/负载/地图匹配质量正相关。
   - 增加 `spike_candidate`、`sustained_motion` 和 `static_context_*` 字段，避免静止时单帧尖峰被误判成持续运动后把证据清掉。
   - `health_events.jsonl` 在 raw/filter/map/filter-status 出现跳变、拒绝、恢复或 bad fitness 时写入事件前后窗口摘要；`RC_HEALTH_MONITOR_EVENT_CONTEXT_CSV=1` 可额外保存逐点上下文。
   - `stop_all.sh` 和 CAN 重启脚本已同步清理该监测进程。

23. `autostart/cleanup_autostart_logs.sh`、`autostart/start_all_terminals.sh`
   - 新增原始运行日志保守清理：每次一键启动时清理超过 `RC_AUTOSTART_LOG_RETENTION_DAYS=30` 天的 `logs/autostart/run_*` 目录。
   - 永远保留最近 `RC_AUTOSTART_LOG_KEEP_RECENT=10` 次、`latest` 当前运行和 `latest_restart` 最近重启目录；清理结果写到当前运行目录的 `log_cleanup.log`。
   - 只清理 `logs/autostart/run_*`，不会删除 `maps/`、`logs/calibration/`、`report_assets/`、PCD 地图或校准报告。`RC_AUTOSTART_LOG_CLEANUP=0` 可关闭，`RC_AUTOSTART_LOG_CLEANUP_DRY_RUN=1` 可只预览。
   - run 目录名追加 `boot_id` 短码和启动脚本 PID，格式类似 `run_20260621_172000_boot_517066fa_1234`，避免系统时间跳变或连续强制重启时复用同一个日志目录。
   - `start_summary.txt` 的关键事件增加 `uptime_sec` 和 `elapsed_sec`，分析开机早期日志时优先看这两个字段，不要只看墙上时间。
   - `RC_AUTOSTART_HEADLESS=auto` 现在默认使用 headless 后台启动核心节点；手动启动时 `RC_AUTOSTART_PLOTJUGGLER=auto` 仍会打开 PlotJuggler，因此正常调试启动和低负载启动的核心链路一致，只差 PlotJuggler。

24. `src/lidar_map_localization`
   - `/map_lidar/quality` 保留前 10 个字段兼容不变，并追加 reason_code、align_ms、raw_scan_points、scan_age_sec、odom_age_sec、output_jump_yaw、map_to_odom 修正量和 overlap_ratio。
   - `RC_MAP_QUALITY_OVERLAP=1` 时才低频计算 scan/map 近邻重合率；默认关闭，避免增加常态 CPU 负载。

25. `autostart/relocalization_ready_notify.py`、`autostart/relocalization_ready_notify.sh`
   - 新增重定位 ready 提示；默认在开机和手动 `RC_LIDAR_MODE=relocalization` 时启动，CAN 自动重启默认不弹窗。
   - 判据为 `/robot/pose_filtered` 刷新正常，`/map_lidar/quality` 连续 accepted/converged，`fitness <= RC_READY_NOTIFY_MAX_FITNESS`，并稳定 `RC_READY_NOTIFY_STABLE_SEC` 秒。
   - ready 后写入 `logs/autostart/<run>/relocalization_ready.json`，并在有图形桌面时弹出“RC LiDAR 重定位完成，现在可以移动小车”的小终端窗口；节点随后退出，常态没有后台负载。
   - `stop_all.sh` 和 CAN 重启清理逻辑已同步处理 `READY_NOTIFY` 进程和窗口。

## 注意事项

- `autostart/backup/` 是旧备份目录，未参与一键启动，里面仍可能保留旧路径。
- 不建议只关闭终端窗口来停止雷达定位栈。需要确认停止时，用 `./autostart/stop_all.sh`；需要确认是否有残留进程时，用 `pgrep -af 'fastlio|livox|rc_serial|rviz2|plotjuggler|robot_tf_offsets|lidar_tf_offset|world_from_fastlio|ekf_node|ros2'`。
- Livox 驱动的 `bind failed` 通常是本机 IP 没配到 JSON 里的 host IP，不是 ROS 编译失败。
- `rc_serial_driver` 依赖系统里存在 SocketCAN 设备。如果没有任何 `can*` / `slcan*` 接口，优先检查 USB-CAN 的 `lsusb`、`dmesg` 和供电/线材/USB 口。
- 如果系统未安装 `robot_localization`，一键启动会自动降级为静态 `body -> body_highrate`。安装 `ros-humble-robot-localization` 后会自动使用 EKF：

```bash
sudo apt update
sudo apt install -y ros-humble-robot-localization
```
- PlotJuggler 自动确认窗口依赖 `xdotool`。如果未安装，PlotJuggler 仍会启动，但需要手动点击 `Warning`、`Start Streaming?`、`Topic Selector` 等弹窗确认。PlotJuggler ROS2 插件没有可靠的完全静默订阅参数，因此本工程采用“启动前写入 `default_topics` + 自动确认弹窗”的低风险方案。
