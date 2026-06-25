# codex 不对这个文件做记录，但是必要错误可修改

cd '/home/orangepi/RC_LIDAR_NOW'
./autostart/start_all_terminals.sh


chromium-browser --proxy-server="http://127.0.0.1:7897"


我现在已经接上了雷达，你测试一下下面的脚本
//低负载启动：
cd '/home/orangepi/RC_LIDAR_NOW'
RC_AUTOSTART_HEADLESS=1 RC_AUTOSTART_PLOTJUGGLER=0 ./autostart/start_all_terminals.sh



# 关残余雷达进程

   cd '/home/orangepi/RC_LIDAR_NOW'
./autostart/stop_all.sh


<!-- 旋转自校正脚本 -->

cd '/home/orangepi/RC_LIDAR_NOW'
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fastlio_world_init rotation_extrinsic_calibrator \
  --duration 40 \
  --rate 50

# Rc 3.4 无头车 TF 树验证：
# map 是 PCD 地图坐标，Y 正方向是无头车世界正方向。
# robot_origin 是机器人业务原点，robot_base_highrate 是机器人当前高频中心。
ros2 run tf2_ros tf2_echo map robot_origin
ros2 run tf2_ros tf2_echo body robot_base
ros2 run tf2_ros tf2_echo body_highrate robot_base_highrate
ros2 run tf2_ros tf2_echo robot_origin robot_base_highrate

建图模块

# 建图
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=mapping ./autostart/start_all_terminals.sh

# 保存地图
cd "/home/orangepi/RC_LIDAR_NOW"
./autostart/save_fast_lio_map.sh

地图默认保存到：
maps/mid360_map.pcd

# 重定位低负载启动：
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
RC_LIDAR_MAP_PATH="/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260624_190419.pcd" \
./autostart/start_all_terminals.sh


# 当前可用版本 rc3.2：小场地直接建图，不裁剪，使用这张地图重定位
# 地图：/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260624_190419.pcd
# 启动前提：车放在建图原点附近，所以使用 RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1
# 重定位调试启动：核心仍然低负载，不开 RViz，但自动打开 PlotJuggler
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=1 \
RC_LIDAR_MAP_PATH="/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260625_064655.pcd" \
./autostart/start_all_terminals.sh

# 启用 stable 候选滤波做 PlotJuggler 对比，但 CAN 仍走默认 /robot/pose_filtered
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=1 \
RC_POSE_FILTER_STABLE=1 \
RC_LIDAR_MAP_PATH="/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260625_064655.pcd" \
./autostart/start_all_terminals.sh

# 确认 stable 更稳且运动不卡顿后，才让 CAN 发送 /robot/pose_filtered_stable


# 如果车确实放回建图开始原点附近：
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
./autostart/start_all_terminals.sh

如果车不是从建图原点启动，需要给粗略初值：
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_INITIAL_GUESS=1 \
RC_RELOCALIZATION_INITIAL_X=0.0 \
RC_RELOCALIZATION_INITIAL_Y=0.0 \
RC_RELOCALIZATION_INITIAL_YAW=0.0 \
./autostart/start_all_terminals.sh


# 备用方案：如果只有大场馆 PCD、没法重新绕小场建图，再裁比赛局部地图
# rc3.2 这次可用方案没有用到裁剪，R2 同步时优先复刻上面“小场直接建图”的流程
cd "/home/orangepi/RC_LIDAR_NOW"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run lidar_map_localization pcd_prepare_local_map \
  --input maps/mid360_map.pcd \
  --output maps/competition_local.pcd \
  --leaf 0.10 \
  --min-x -4.0 --max-x 4.0 \
  --min-y -3.0 --max-y 5.0 \
  --margin 2.0

# 用比赛局部地图做低负载重定位，CAN 默认等待 /map_lidar/quality 稳定后才发
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_LIDAR_MAP_PATH="/home/orangepi/RC_LIDAR_NOW/maps/competition_local.pcd" \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
./autostart/start_all_terminals.sh

# 如果只是临时排查 CAN 链路，不想等待重定位 ready，可以短时间关闭门控
RC_CAN_REQUIRE_READY=0 ./autostart/start_all_terminals.sh



chromium-browser --proxy-server="http://127.0.0.1:7897"


# debug脚本

cd "/home/orangepi/RC_LIDAR_NOW"
./autostart/stop_all.sh

RC_THERMAL_POSE_MONITOR=1 \
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
./autostart/start_all_terminals.sh


分别维护readme和更新日志两个内容，更新日志大概要做到另一个codex在没有上下文的情况下知道要干什么就可以了，不用告诉他具体怎么做。而是让他知道有这件事。（但是对事的记录要详细，尽量不要漏，甚至就可以以我每次给你发的话为主题参照）

我刚刚重启了两次，现在是第三次的开机，应该此时还有一个正在运行的雷达程序。理论上，现在应该有两个已经新生成的低负载日志和一个正在运行的高负载日志，你帮我判断一下前两次的低负载日志。分析一下具体坐标情况，以及波动情况

我刚刚在静止状态下不同情况低功耗启动了两次。都维持静止，你帮我对比这两次有没有区别？现在是第三次的开机，


跑通现在的CAN链路，另外，现在的滤波算法开销很大，除去VSCODE和clash也占满了CPU,先1后2


# 调试负载注意

用 Codex 查问题时，通常必须开 VS Code 和 Clash，有时还会开 QQ、浏览器或 PlotJuggler，这会显著提高 CPU/温度负载；低功耗开机自启不会开这些，所以不能直接把 Codex 调试状态下的 2-5cm 抖动当成生产低负载状态。

判断滤波问题时优先做 headless 对照：

cd "/home/orangepi/RC_LIDAR_NOW"
./autostart/stop_all.sh

RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
RC_POSE_FILTER_STABLE=0 \
RC_HEALTH_MONITOR_INTERVAL=1.0 \
RC_LIDAR_MAP_PATH="/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260625_064655.pcd" \
./autostart/start_all_terminals.sh

看日志时先分清：
- /robot/pose 原始位姿是否已经跳；
- /map_lidar/quality 的 fitness 和 map_quality_jump_xy_m 是否同步变差；
- /robot/pose_filtered 是否只是被原始位姿拉着走。


# temperature

cat


# 常用控制变量速查（精简版）

只保留最常用的 10 个以内。现场调车优先低负载：headless、不开 PlotJuggler、指定小场地图、开启 stable 候选。

## 推荐开机低负载重定位

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
RC_POSE_FILTER_STABLE=1 \
RC_LIDAR_MAP_PATH="/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260625_064655.pcd" \
./autostart/start_all_terminals.sh
```

## 最常用变量

1. `RC_LIDAR_MODE=mapping|relocalization`
   `mapping` 建图，`relocalization` 重定位。调车/开机自启默认用 `relocalization`。

2. `RC_LIDAR_MAP_PATH=...`
   指定重定位使用哪张 PCD 地图。换地图时最常改这个。

3. `RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1|0`
   `1` 表示启动点接近地图/机器人业务原点；当前小场地图方案常用 `1`。

4. `RC_AUTOSTART_HEADLESS=1|0`
   `1` 无头低负载后台启动；`0` 更偏人工调试。开机自启/无显示器用 `1`。

5. `RC_AUTOSTART_PLOTJUGGLER=1|0`
   `1` 自动打开 PlotJuggler 看曲线；`0` 不打开，低负载调车默认用 `0`。

6. `RC_POSE_FILTER_STABLE=1|0`
   `1` 启动 stable 滤波候选 `/robot/pose_filtered_stable`；`0` 关闭 stable 候选。

7. `RC_POSE_FILTER_SELECTED=normal|stable|beta`
   选择 CAN/生产链路使用哪个滤波输出。选 `stable` 时要同时 `RC_POSE_FILTER_STABLE=1`。

8. `RC_CAN_REQUIRE_READY=1|0`
   `1` CAN 等重定位 ready 后再发；正式调车建议 `1`。`0` 只适合排查 CAN 通不通。

9. `RC_HEALTH_MONITOR=1|0`
   `1` 写健康日志，记录温度、负载、地图质量和位姿；开销小，建议默认 `1`。

10. `RC_ROBOT_POSE_SOURCE=odom|tf`
    `/robot/pose` 来源。当前生产默认 `odom`，从 `/map_lidar/odom` 同帧计算机器人中心；`tf` 只用于对比旧 TF 拼接路径。

当前默认生产链路：

```text
/map_lidar/odom
  -> pose_from_odom 加 body -> robot_base 外参
  -> /robot/pose
  -> /robot/pose_filtered 或所选滤波输出
  -> CAN
```
