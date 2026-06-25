# Rc 4.11: Map Localizer Only Keeps Latest Input

## Summary

这份日志记录一次关键修正：`map_localizer` 不再让 scan/odom 历史输入排队进入重定位计算链路，而是默认只保留最新输入。

对应提交：

```text
bfb2042c8ca2fe9139f1f9e7cec07c26401c1886
Rc 4.11: keep latest map localization inputs
```

这个改动的核心目标是解决无头车旋转时的“历史点云/历史 odom 回放”问题。旧策略允许 map localizer 在算不过来时继续处理较旧输入，导致地图修正结果滞后进入 `/map_lidar/odom`、`/robot/pose`、yaw fusion 和 CAN 链路。现策略改为面向实时控制：算不过来时丢旧输入，优先算最新输入。

## Problem

重写无头车 TF 树和 `/robot/pose` 计算链路后，用户实车观察到：

- 机器人已经停止旋转，但 PlotJuggler 里的 yaw 还会继续变化一段时间。
- 通过 `/robot/yaw_fusion/status` 对比时，`fast_yaw` 基本实时，`map_ref_yaw` 和 `fused_yaw` 仍像是在慢慢追历史。
- 降低 map correction alpha 后，无头车旋转更灵敏，但地图把漂移拉回来的速度变慢。
- 这说明问题不只是滤波，也不只是 alpha 参数，而是 map localizer 的输入和输出可能已经滞后于当前机器人状态。

旧代码里 scan 订阅使用 `SensorDataQoS()`，odom 订阅显式使用队列深度 `20`：

```cpp
scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    scan_topic_, rclcpp::SensorDataQoS(), ...);

odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 20, ...);
```

对普通数据处理来说，保留多帧历史可能合理；但对无头车实时控制来说，如果地图匹配速度低于输入频率，历史输入排队会把“过去的姿态修正”延迟注入当前控制链路。用户说的“点云历史队列有 20 个信息窗口”主要对应这个旧 odom queue depth，同时 scan 侧也存在保留旧 sensor data 的风险。

## Key Insight

map localizer 不应该像离线处理一样补算每一帧历史点云。对这套车来说，它的职责是给 FAST-LIO 当前位姿提供地图约束，而不是把历史修正一帧帧回放给电控。

因此更合适的实时策略是：

```text
正在计算的一帧继续算完；
等待队列里只保留最新的一帧；
如果又来了更新的点云/odom，就覆盖旧的等待输入；
算完后用最新可用输入继续下一轮。
```

这样即使单次 NDT/ICP 匹配需要几十到一百多毫秒，也不会把 1-2 秒前的输入排队播放到 yaw/pose 修正里。

## Rejected Ideas

### Increase Filtering Strength

一开始怀疑是滤波造成 yaw 和 XY 延迟。后来通过 `/robot/yaw_fusion/status` 看到 `fast_yaw` 很实时，而 map 相关 yaw 慢慢追，说明根因不在 pose filter 本身。继续加大滤波只会掩盖问题，还可能增加 CAN 延迟。

### Only Lower Map Correction Alpha

把 `RC_ROBOT_POSE_YAW_FUSION_MAP_CORRECTION_ALPHA` 降到 `0.03` 后，旋转响应确实更好，但地图校正变慢，FAST-LIO 的累积误差需要更久才能拉回来。这是一个缓解手段，不是根因修复。

### Drop Results Only By Timestamp

用户担心：如果 map localizer 本身一直算得慢，算完就因为 timestamp 太旧被丢弃，可能永远无法校正。

因此 Rc 4.11 没有先做“按时间戳硬丢弃输出”。第一步采用更稳的方案：防止输入层积压历史，只保留最新 scan/odom；输出仍接受，但通过健康日志记录 `scan_age_sec`、`odom_age_sec`、`align_ms` 来判断是否仍然过慢。

## Final Design

### 1. Input Queue Depth Defaults To 1

`map_localizer` 新增参数：

```text
input_queue_depth: 1
```

代码里 scan 和 odom 统一使用：

```cpp
auto input_qos = rclcpp::QoS(rclcpp::KeepLast(input_queue_depth_))
                     .best_effort()
                     .durability_volatile();
```

默认生产配置：

```bash
RC_MAP_LOCALIZER_INPUT_QUEUE_DEPTH=1
```

语义是：如果 map matching 算不过来，旧 scan/odom 被最新输入覆盖，不再排队补算历史。

### 2. Keep Latest Scan/Odom In The Node

回调里仍然只维护：

```cpp
latest_scan_
latest_scan_stamp_
latest_odom_
```

timer 触发时复制当前最新 scan/odom 做一次匹配。这样 map localizer 不需要维护自己的多帧历史队列，行为更接近“最新状态校正器”。

### 3. Map Correction Alpha Reduced

同一个提交把 yaw fusion 的地图校正默认值从 `0.25` 降为 `0.05`：

```bash
RC_ROBOT_POSE_YAW_FUSION_MAP_CORRECTION_ALPHA=0.05
```

这不是队列修复本身，但和队列修复配套。新的语义是：

- FAST-LIO yaw 负责快速响应旋转。
- map yaw 负责低频、弱力度地把方向拉回全局地图。
- 即使 map 输出仍有一个计算周期延迟，也不应该强力拖慢无头车 yaw。

### 4. Diagnostics Kept In Quality Logs

没有删除诊断，反而保留并强化以下字段的意义：

```text
scan_age_sec
odom_age_sec
align_ms
map_to_odom_delta_xy_m
map_to_odom_delta_yaw_deg
quality reason
```

这些字段用于判断后续是否还需要第二层输出门控，例如“匹配结果太旧时只记录、不参与 correction”。

## Affected Files

核心代码：

```text
src/lidar_map_localization/src/map_localizer.cpp
src/lidar_map_localization/config/map_localizer.yaml
src/lidar_map_localization/launch/map_localizer.launch.py
autostart/map_localizer.sh
```

配套 yaw correction 默认值：

```text
autostart/pose_interface_relocalization.sh
src/lidar_localization/config/lidar_pose_interface.yaml
src/lidar_localization/launch/lidar_pose_from_odom.launch.py
src/lidar_localization/lidar_localization/pose_from_odom.py
```

文档同步：

```text
readme.md
R2_MIGRATION_RUNBOOK.md
change_logs/2026-06-25-rc4-tf-tree-to-odom-pose-source.md
```

## Runtime Behavior

Rc 4.11 之后，用户继续实测发现：

- `/robot/pose` 原始位姿本身已经明显变稳。
- 静止时不滤波的误差可以偶发维持在约 `6mm`。
- `/robot/pose` 和 `/robot/pose_filtered` 的稳定性差异变小，说明主要收益来自上游 map/pose 输入变稳，而不是下游滤波强行压抖动。

这符合预期：一旦旧输入不再回放，滤波器收到的 raw pose 本身就更接近当前真实状态。

## Validation Commands

静态检查：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"

bash -n \
  autostart/map_localizer.sh \
  autostart/pose_interface_relocalization.sh

python3 -m py_compile \
  src/lidar_map_localization/launch/map_localizer.launch.py \
  src/lidar_localization/launch/lidar_pose_from_odom.launch.py
```

编译：

```bash
colcon build --symlink-install --packages-select \
  lidar_map_localization lidar_localization
```

Dry-run 检查启动参数：

```bash
RC_AUTOSTART_DRY_RUN=1 \
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
./autostart/start_all_terminals.sh
```

运行时检查：

```bash
tail -n 120 logs/autostart/latest/MAP_LOCALIZER.log
```

期望看到：

```text
input_queue_depth=1
```

检查 topic 频率：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic hz /map_lidar/odom
ros2 topic hz /robot/pose
ros2 topic hz /robot/pose_filtered
```

观察质量指标：

```bash
ros2 topic echo /map_lidar/quality --once
```

重点看 `scan_age_sec`、`odom_age_sec`、`align_ms` 是否明显偏大。

## Migration Notes For R2 Or Other Repos

可以直接复用：

```bash
RC_MAP_LOCALIZER_INPUT_QUEUE_DEPTH=1
RC_ROBOT_POSE_YAW_FUSION_MAP_CORRECTION_ALPHA=0.05
```

迁移时必须确认：

- r2 的 map localizer 是否也是用 scan + odom 做 NDT/ICP。
- r2 的 executor 是否单线程或多线程；多线程下还要确认互斥锁和正在计算帧的所有权。
- r2 的 `/map_lidar/quality` 是否已经包含 scan/odom age 和 align time。
- r2 的 CAN 是否直接使用 `/robot/pose_filtered` 或另一个 pose topic。

不要照搬：

- 当前 Orange Pi 的地图路径。
- 当前场地的 map correction alpha 最终值。
- 当前机器人的机械外参。
- 当前健康日志里的实验数据。

如果 r2 后续也出现“车停了，yaw/pose 还慢慢追”的问题，优先检查输入队列和 `scan_age_sec/odom_age_sec/align_ms`，而不是先加大滤波。

## Known Limits

`input_queue_depth=1` 解决的是历史输入积压，不会让单次 NDT/ICP 计算凭空变快。

如果单次匹配仍然持续超过控制可接受延迟，后续方向是：

- 降低 map localizer 计算频率，但保证算的是最新输入。
- 降低 scan 下采样后点数或调大 voxel leaf size。
- 对 map correction 做更明确的延迟补偿。
- 在输出层增加“过旧结果只记录、不修正”的可配置门控。

## User Question Appendix

以下内容按讨论顺序保留用户关于本改动的关键问题、判断和约束。长代码、长日志和截图不逐字保存，只记录与本主题相关的含义。

1. 用户观察到无头车旋转时，降低 map correction alpha 到约 `0.03` 后确实更能转着走，但又出现 map 把 FAST-LIO 漂移拉回来的时间变长的问题。

2. 用户判断：原始 odom 实时性很强，map localizer 可能在算旧点云，旧点云结果把机器人的旋转往回拖；alpha 大时拖得明显，alpha 小时拖得轻但修正慢。

3. 用户提出：map 节点不应该反复计算历史 stamp，而应该丢弃历史坐标，只计算当前机器人的点云。

4. 用户澄清术语理解：scan 是当前扫描点云，map 是全局重定位节点，odom 是 FAST-LIO 自身定位；本质上应该让 map 节点丢弃太旧的点云，只努力算最新点云。

5. 用户提醒一个风险：如果 map localizer 本身真的很慢，每次算完都被判定太旧而丢掉，就可能永远纠正不了。因此不能只靠时间戳硬丢弃，需要考虑“至少能持续校正”的策略。

6. 用户提出最终策略雏形：维护长度为 1 的输入队列；一帧正在算，另一帧作为最新等待输入被新输入不断覆盖；算完后不先做严格时间戳丢弃，而是用可调 correction alpha 进行修正。

7. 用户追问：这样改后 XY 是否也会受益。结论是会受益，因为旧 map 修正不仅影响 yaw，也会影响 `/map_lidar/odom` 里的 x/y。

8. 用户后续实测发现：纠正点云历史队列后，即使不滤波的 `/robot/pose` 静态误差也能到约 `6mm`，且和滤波后差距不大，说明上游 raw pose 已经明显变稳。

9. 用户当前要求：围绕“点云历史队列”整理一份 change log，并特别指出旧实现里 odom queue depth 曾经是 `20`，这对实时无头车控制非常不合理。
