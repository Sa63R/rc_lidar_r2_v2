# RC LIDAR ROS 2 工作区编译和运行说明

本文记录当前 `src/` 工作区的编译环境、构建命令、运行顺序、硬件前置条件，以及为了适配当前环境做过的源码修改。

## 当前已验证环境

- 系统：Ubuntu 22.04.5 LTS, aarch64
- ROS：ROS 2 Humble
- 工作区路径：`/home/orangepi/RC_LIDAR_NOW!!!`
- 已存在关键工具：`colcon`、`ros2`、`rosdep`
- 已安装 PlotJuggler：`ros-humble-plotjuggler`、`ros-humble-plotjuggler-ros`
- 已存在 Livox-SDK2：
  - `/usr/local/include/livox_lidar_api.h`
  - `/usr/local/include/livox_lidar_def.h`
  - `/usr/local/lib/liblivox_lidar_sdk_shared.so`
  - `/usr/local/lib/liblivox_lidar_sdk_static.a`

当前机器上 `rosdep` 还没有初始化，且当前用户没有免密 sudo，所以部分系统级安装需要手动输入 sudo 密码。PlotJuggler 自动确认弹窗需要 `xdotool`。

## 包结构

`src/` 下包含 5 个 ROS 2 包：

- `livox_ros_driver2`：Livox 雷达驱动，发布 `/livox/lidar` 和 `/livox/imu`
- `fast_lio`：FAST-LIO 建图定位节点
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
cd "/home/orangepi/RC_LIDAR_NOW!!!"
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
Summary: 5 packages finished
```

可执行文件检查：

```bash
for p in fast_lio livox_ros_driver2 rc_serial_driver fastlio_world_init my_static_tf; do
  echo "== $p =="
  ros2 pkg executables "$p"
done
```

当前验证到的可执行文件：

```text
fast_lio fastlio_mapping
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

为了保证开机后一键启动时不再要求输入 sudo 密码，先运行一次：

```bash
cd "/home/orangepi/RC_LIDAR_NOW!!!"
./autostart/install_one_click_network.sh
```

这个脚本会把 `eth0` 的 `192.168.1.5/24` 写进 NetworkManager，并安装 udev 规则，让 USB-CAN 枚举成 `can*` 时自动配置 `1000000` bit/s 和 `restart-ms 100`。

设置桌面开机自启：

```bash
cd "/home/orangepi/RC_LIDAR_NOW!!!"
./autostart/install_desktop_autostart.sh
```

这会在 `/home/orangepi/.config/autostart/` 安装一个 `.desktop` 文件。它会在 `orangepi` 用户登录图形桌面后自动运行 `autostart/start_all_terminals.sh`，日志写到：

```text
logs/autostart/xdg_autostart.log
```

开机自启默认使用低负载模式：

```text
RC_AUTOSTART_HEADLESS=1
RC_AUTOSTART_PLOTJUGGLER=0
RC_AUTOSTART_TF_COMPARE=auto
RC_AUTOSTART_LIDAR_XYZR=auto
```

也就是说，开机后会启动定位、TF、Livox、CAN 等核心链路，但不会弹 RViz/PlotJuggler 界面；TF 对比发布节点和点云调试打印也会随 PlotJuggler/headless 策略默认跳过，避免香橙派 CPU 被调试工具抢走。需要临时开图形调试时，手动运行一键脚本并覆盖这些环境变量即可。

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
cd "/home/orangepi/RC_LIDAR_NOW!!!"
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
cd "/home/orangepi/RC_LIDAR_NOW!!!"
RC_AUTOSTART_DRY_RUN=1 ./autostart/start_all_terminals.sh
```

确认输出里启动项都显示 `would launch` 后，再执行真正的一键启动：

```bash
cd "/home/orangepi/RC_LIDAR_NOW!!!"
./autostart/start_all_terminals.sh
```

日志会写到：

```text
/home/orangepi/RC_LIDAR_NOW!!!/logs/autostart/latest
```

一键启动默认会先执行一次旧进程清理，避免重复运行脚本后出现两套 Livox / FAST-LIO / EKF / CAN 发送节点同时发布。清理日志在：

```text
logs/autostart/latest/prestop.log
```

如果只是想手动停止当前雷达定位栈：

```bash
cd "/home/orangepi/RC_LIDAR_NOW!!!"
./autostart/stop_all.sh
```

一般不要只靠关闭终端窗口来停止程序；低负载、开机自启、CAN 重启等 headless 模式会把进程放到后台，关闭终端不会可靠停止它们。确实需要跳过启动前清理时，可以临时设置：

```bash
RC_AUTOSTART_SKIP_PRESTOP=1 ./autostart/start_all_terminals.sh
```

默认会打开这些终端：

```text
TF_STATIC1       body -> base_link
TF_STATIC2       world -> init
TF_STATIC3       body_highrate -> base_link_highrate
LIVOX            Livox MID360 driver
FASTLIO          FAST-LIO mapping, rviz 默认 true
HIGHRATE_BODY    robot_localization EKF；未安装时降级为 body -> body_highrate 静态 TF
world_tf         fastlio_world_init
TF_ROBOT_OFFSET  发布 init -> base_link / base_link_highrate 的 XYZ 对比 topic
TF_LIDAR_OFFSET  发布 world -> body 的 XYZ 对比 topic
CAN_SERIAL       rc_serial_driver
LIDAR_XYZR       点云调试打印
PLOTJUGGLER      打开 TF Compare 布局并订阅 /tf_compare topic
```

手动启动时 FAST-LIO 默认会同时打开 RViz。如果想关闭 RViz：

```bash
FASTLIO_RVIZ=false ./autostart/start_all_terminals.sh
```

如果使用 headless 模式，FAST-LIO 默认不打开 RViz：

```bash
RC_AUTOSTART_HEADLESS=1 RC_AUTOSTART_PLOTJUGGLER=0 ./autostart/start_all_terminals.sh
```

PlotJuggler 默认在手动启动时打开，开机自启和 CAN 自动重启时默认不打开，避免反复弹 GUI 或占用香橙派 CPU。可以用下面的环境变量覆盖：

```bash
RC_AUTOSTART_PLOTJUGGLER=0 ./autostart/start_all_terminals.sh
RC_AUTOSTART_PLOTJUGGLER=1 RUN_REASON=boot ./autostart/start_all_terminals.sh
RC_AUTOSTART_PLOTJUGGLER=1 RUN_REASON=can_restart ./autostart/start_all_terminals.sh
```

TF 对比发布节点默认只在 PlotJuggler 会启动时自动启动；`LIDAR_XYZR` 点云调试打印默认只在非 headless 模式启动。性能测试或实车调车时建议保持默认低负载策略。需要单独打开调试节点时：

```bash
RC_AUTOSTART_TF_COMPARE=1 RC_AUTOSTART_LIDAR_XYZR=1 \
  RC_AUTOSTART_HEADLESS=1 RC_AUTOSTART_PLOTJUGGLER=0 \
  ./autostart/start_all_terminals.sh
```

PlotJuggler 的 ROS2 插件会弹出确认窗口，本工程用 `xdotool` 自动按确认。如果没有安装：

```bash
sudo apt install -y xdotool
```

## 自转外参校准

如果机械给出的雷达相对底盘中心坐标存在微小偏差，机器人原地自转时，`init -> base_link_highrate` 的 XY 会出现本不该有的小圆波动。可以用自转校准工具记录这段数据并自动计算建议修正值。

先正常启动低负载定位栈，等待 `world_tf` 初始化完成，然后运行：

```bash
cd "/home/orangepi/RC_LIDAR_NOW!!!"
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
init -> body_highrate
init -> base_link_highrate
body_highrate -> base_link_highrate
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
x=-0.240, y=0.287302, z=-0.760, yaw=pi
```

其中 `x` 的拟合修正量约 `0.01 mm`，低于实际安装和定位噪声，因此保持 `-0.240` 不变；`y` 按建议值从 `0.315` 修正为 `0.287302`。这组三处静态 TF 必须保持一致：

```text
world -> init
body -> base_link
body_highrate -> base_link_highrate
```

默认只输出建议值，不会自动修改 TF。确认报告可信后，把 `summary.json` 和 `samples.csv` 发出来，再同步修改三处静态 TF：

```text
src/my_static_tf/launch/map_to_init.launch.py
src/my_static_tf/launch/lidar_to_base.launch.py
src/my_static_tf/launch/lidar_to_base_highrate.launch.py
```

本校准只修正 XY；`z`、`roll`、`pitch`、`yaw=pi` 保持不变。

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
ros2 topic echo /tf_compare/robot_init_to_base_link --once
ros2 topic echo /tf_compare/robot_init_to_base_link_highrate --once
ros2 topic echo /tf_compare/lidar_world_to_body --once
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo camera_init body
ros2 run tf2_ros tf2_echo world init
ros2 run tf2_ros tf2_echo body base_link
ros2 run tf2_ros tf2_echo body_highrate base_link_highrate
ros2 run tf2_ros tf2_echo init base_link
ros2 run tf2_ros tf2_echo init base_link_highrate
```

## 本次修改

本次为了让当前环境能够稳定编译和运行，做了以下小修改，没有改变核心算法逻辑。

1. `src/fastlio_world_init/fastlio_world_init/world_from_fastlio_init.py`
   - 修复 Ctrl-C 或超时退出时重复调用 `rclpy.shutdown()` 导致的异常。
   - 把日志里的静态 TF 发布方向改成实际方向：`source_parent_frame -> world_frame`。

2. `src/rc_serial_driver/launch/serial_driver.launch.py`
   - 原 launch 文件加载 `config/serial_driver.yaml`，但实际配置文件名是 `rc_serial_driver.yaml`。
   - 已改为加载 `config/rc_serial_driver.yaml`，否则节点会启动但不会读取配置文件。

3. `autostart/*.sh`
   - 统一通过 `autostart/common_env.sh` 加载当前工作区环境。
   - 去掉旧路径 `/home/rc/...`、`/home/rc2/...`、`~/jwt/...`。
   - `start_all_terminals.sh` 改为自动定位当前 `autostart/`，日志写入本工作区的 `logs/autostart/`。
   - `serial_driver.sh` 会尝试免密 sudo 拉起 CAN；如果没有权限，会打印手动命令并继续启动节点。
   - `highrate_body.sh` 在缺少 `robot_localization` 时发布 `body -> body_highrate` 静态 TF 作为降级兜底。

4. `src/my_static_tf/launch/lidar_to_base_highrate.launch.py`
   - 新增高频 base_link 静态 TF：`body_highrate -> base_link_highrate`，用于修复原脚本引用了不存在 launch 文件的问题。

5. `src/my_static_tf/launch/map_to_init.launch.py`
   - 把静态 TF 从 `lidar -> init` 调整为正常运行树需要的 `world -> init`，使 `init` 能连接到 `camera_init/world/body_highrate` 主树。

6. `src/my_static_tf/launch/*.launch.py`
   - 按雷达安装位置更新机器人中心相对雷达坐标系的静态外参。
   - 根据 `logs/calibration/rotation_extrinsic_20260619_175422/` 的自转校准结果修正 Y 方向外参。
   - 当前统一参数为：`x=-0.240, y=0.287302, z=-0.760, roll=0, pitch=0, yaw=pi`。
   - `world -> init`、`body -> base_link`、`body_highrate -> base_link_highrate` 三套 TF 使用同一组外参。

7. `autostart/world_tf.sh`
   - `world` 初始化保持为“先等待 10 秒，再对 FAST-LIO 的 `camera_init -> body` 采样平均 3 秒”，用于避开刚启动时的不稳定状态。

8. `src/fastlio_world_init/fastlio_world_init/robot_tf_offsets.py`
   - 新增 TF 对比 topic：`/tf_compare/robot_init_to_base_link` 和 `/tf_compare/robot_init_to_base_link_highrate`。
   - 分别发布 `init -> base_link` 与 `init -> base_link_highrate` 的 XYZ 平移。

9. `src/fastlio_world_init/fastlio_world_init/lidar_tf_offset.py`
   - 新增 TF 对比 topic：`/tf_compare/lidar_world_to_body`。
   - 发布 `world -> body` 的 XYZ 平移，保持雷达原始坐标轴。

10. `src/fastlio_world_init/config/tf_compare_plotjuggler.xml`
    - 新增 PlotJuggler 布局，自动预选三个 `/tf_compare/...` topic。
    - 布局中按 X/Y/Z 三个窗口对比机器人低频、机器人高频、雷达三条曲线。

11. `autostart/robot_tf_offsets.sh`、`autostart/lidar_tf_offset.sh`、`autostart/plotjuggler_tf_compare.sh`
    - 一键启动时自动启动 TF 对比发布节点和 PlotJuggler。
    - `plotjuggler_tf_compare.sh` 会等待 `/tf_compare/...` topic 出现，再打开 PlotJuggler。

12. `src/fastlio_world_init/fastlio_world_init/rotation_extrinsic_calibrator.py`
    - 新增自转外参校准工具。
    - 原地自转时记录 `init -> body_highrate`、`init -> base_link_highrate`、`body_highrate -> base_link_highrate`。
    - 额外记录 `camera_init -> body`、`world -> body` 和 `/Odometry` 的 yaw 诊断，避免只看单一路径时误判旋转圈数。
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

16. `autostart/install_one_click_network.sh`
   - 一次性配置免密码一键启动所需的网络环境。
   - 持久化 `eth0 -> 192.168.1.5/24`。
   - 安装 udev 规则，USB-CAN 枚举为 `can*` 时自动拉起 CAN 口。

17. `autostart/boot_start.sh`、`autostart/rc_lidar_autostart.desktop`、`autostart/install_desktop_autostart.sh`
   - 新增图形桌面开机自启入口。
   - 登录 `orangepi` 桌面后自动运行 `autostart/start_all_terminals.sh`。
   - 自启日志写到 `logs/autostart/xdg_autostart.log`。

18. `autostart/stop_all.sh`、`autostart/start_all_terminals.sh`
   - 新增统一停止脚本，用于清理本项目启动的 Livox、FAST-LIO、静态 TF、EKF、CAN 发送、RViz、PlotJuggler、自转校准等进程。
   - `start_all_terminals.sh` 默认会在真正启动前调用 `stop_all.sh --quiet`，防止重复启动导致多个 `/odometry/filtered` publisher 或重复 TF。
   - `RC_AUTOSTART_DRY_RUN=1` 时不会执行清理；需要临时跳过可设置 `RC_AUTOSTART_SKIP_PRESTOP=1`。

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
- PlotJuggler 自动确认窗口依赖 `xdotool`。如果未安装，PlotJuggler 仍会启动，但需要手动点击 `Warning`、`Start Streaming?`、`Topic Selector` 等弹窗确认。
