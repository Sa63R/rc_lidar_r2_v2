# Ubuntu 24.04 / ROS 2 Jazzy

此分支基于 main `3d3ab89721bbbb90d735100fbb0651d2ebccc598`，在 Ubuntu 24.04 amd64、ROS 2 Jazzy、系统 Python 3.12、PCL 1.14 上验证。旧分支有独立提交，予以保留。

## 编译

先安装 ROS 2 Jazzy 和 Livox SDK2；本机复用 `/usr/local/lib/liblivox_lidar_sdk_shared.so`。工作区有 Conda 或相机 SDK 环境时，使用系统 Python，并避免旧版 libusb 进入 PCL 进程。

```bash
cd /path/to/rc_lidar_r2_v2
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
  --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

`autostart/common_env.sh` 默认 Jazzy；其他发行版用 `RC_ROS_DISTRO` 或完整的 `RC_ROS_SETUP` 选择。工作区用 `RC_LIDAR_WORKSPACE` 配置。该脚本只在自己的进程环境中过滤 `/opt/MVS/lib/*`，不修改相机软件。Livox 历史 LVX 路径可用 `RC_LIVOX_LVX_PATH` 指定。

## 无硬件回放入口

```bash
source autostart/common_env.sh
export ROS_DOMAIN_ID=73
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mkdir -p maps
ros2 launch ./tools/offline.launch.py mode:=mapping \
  config:=/absolute/path/mid360.yaml map_path:=/absolute/path/demo_map.pcd
# 另一个同环境终端：必须是包含真实逐点时间和 IMU 的 CustomMsg 录包
ros2 bag play /absolute/path/bag --clock 100 --delay 3 --rate 0.5
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

停止整套节点后再重新回放；不要在同一进程中把时间跳回起点。重定位使用 `mode:=localization` 和同场景保存的地图。入口不启动 Livox 驱动、CAN、网口配置或系统校时操作，所有节点使用 `use_sim_time`。工具界面也应使用相同 domain 和模拟时间。

离线入口以同一录包起点为初值，教学默认 ICP、0.25 m 体素、fitness 门限 0.20。实车默认配置及 CAN 协议不变。它不是任意初始位置的全局定位。离线入口的 body→robot_base 外参为零，保留测量时间、关闭业务归零；公共录包不代表本队机器人安装尺寸。

## 验证

七个包编译成功。`lidar_localization` 的 55 个功能测试通过；连同 `fastlio_world_init` 功能测试共 57 个通过、1 个跳过。补充 `tests_require` 和测试依赖后，colcon 能发现前者的 pytest 测试。

```bash
source autostart/common_env.sh
colcon test --packages-select lidar_localization
python3 -m pytest src/lidar_localization/test src/fastlio_world_init/test \
  -k 'not flake8 and not pep257' -q
```

全量 `colcon test` 仍有原仓库的 flake8、pep257、uncrustify 等风格检查失败；这不是全量测试全绿的分支。未修改的 main 源码也复现了 Python 风格检查失败。未用大范围格式化掩盖适配改动。

真实 MID-360 离线数据已完成建图、保存地图、重定位和重复启动验证；这不构成实机网口、CAN、硬件授时或现场定位精度验证。教学 PDF、私有代码分析、录包、截图和详细日志保留本机，不纳入此公共仓库。
