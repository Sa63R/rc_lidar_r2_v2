# RC LIDAR ROS 2 工作区编译和运行说明

本文记录当前 `src/` 工作区的编译环境、构建命令、运行顺序、硬件前置条件，以及为了适配当前环境做过的源码修改。

## 当前已验证环境

- 系统：Ubuntu 22.04.5 LTS, aarch64
- ROS：ROS 2 Humble
- 工作区路径：`/home/orangepi/RC_LIDAR_NOW!!!`
- 已存在关键工具：`colcon`、`ros2`、`rosdep`
- 已存在 Livox-SDK2：
  - `/usr/local/include/livox_lidar_api.h`
  - `/usr/local/include/livox_lidar_def.h`
  - `/usr/local/lib/liblivox_lidar_sdk_shared.so`
  - `/usr/local/lib/liblivox_lidar_sdk_static.a`

当前机器上 `rosdep` 还没有初始化，且当前用户没有免密 sudo，所以本次没有执行系统级安装。现有系统依赖已经足够完成编译。

## 包结构

`src/` 下包含 5 个 ROS 2 包：

- `livox_ros_driver2`：Livox 雷达驱动，发布 `/livox/lidar` 和 `/livox/imu`
- `fast_lio`：FAST-LIO 建图定位节点
- `my_static_tf`：静态 TF launch
- `fastlio_world_init`：根据 FAST-LIO 初始 TF 生成 world 初始化 TF 的 Python 节点
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
- 雷达 IP：`192.168.1.155`

当前机器实际活动网口是 `192.168.178.148/24`，不在 `192.168.1.0/24` 网段，所以启动 Livox 驱动时出现过：

```text
bind failed
Failed to init livox lidar sdk.
```

这不是编译问题，是网口 IP 和配置文件不匹配。运行前需要把接雷达的网口配置到 `192.168.1.5/24`，或者把 `MID360_config.json` 里的 host IP 改成实际雷达网段内的本机 IP。

临时配置示例：

```bash
ip -brief addr
sudo ip link set <雷达网口名> up
sudo ip addr add 192.168.1.5/24 dev <雷达网口名>
ping 192.168.1.155
```

如果提示地址已存在或网段冲突，先用 `ip addr show <雷达网口名>` 检查，再清理旧地址。

### CAN 接口

`rc_serial_driver` 默认使用：

```text
can_interface: can0
can_bitrate: 1000000
```

当前机器检测到 `can0` 和 `can1` 都存在，但状态是 `DOWN/STOPPED`。运行串口/CAN 节点前需要先拉起 `can0`：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

如果实际使用 `can1`，修改：

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

确认输出里 9 个启动项都显示 `would launch` 后，再执行真正的一键启动：

```bash
cd "/home/orangepi/RC_LIDAR_NOW!!!"
./autostart/start_all_terminals.sh
```

日志会写到：

```text
/home/orangepi/RC_LIDAR_NOW!!!/logs/autostart/latest
```

默认会打开这些终端：

```text
TF_STATIC1       body -> base_link
TF_STATIC2       lidar -> init
TF_STATIC3       body_highrate -> base_link_highrate
LIVOX            Livox MID360 driver
FASTLIO          FAST-LIO mapping, rviz 默认 true
HIGHRATE_BODY    robot_localization EKF；未安装时降级为 body -> body_highrate 静态 TF
world_tf         fastlio_world_init
CAN_SERIAL       rc_serial_driver
LIDAR_XYZR       点云调试打印
```

FAST-LIO 默认会同时打开 RViz。如果想关闭 RViz：

```bash
FASTLIO_RVIZ=false ./autostart/start_all_terminals.sh
```

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
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo camera_init body
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

5. `src/rc_serial_driver/config/rc_serial_driver.yaml`
   - 增加 `restart_script`，指向当前工作区的 `autostart/restart_lidar_stack.sh`。

## 注意事项

- `autostart/backup/` 是旧备份目录，未参与一键启动，里面仍可能保留旧路径。
- Livox 驱动的 `bind failed` 通常是本机 IP 没配到 JSON 里的 host IP，不是 ROS 编译失败。
- `rc_serial_driver` 在 `can0` down 时会反复尝试重连并打印 `Network is down`，先把 CAN 接口拉起来再启动即可。
- 当前系统未安装 `robot_localization`，一键启动会自动降级为静态 `body -> body_highrate`。安装 `ros-humble-robot-localization` 后会自动使用 EKF。
- 当前工作区不是顶层 git 仓库；如果后续要追踪修改，建议先初始化 git 或把修改后的压缩包单独保存。
