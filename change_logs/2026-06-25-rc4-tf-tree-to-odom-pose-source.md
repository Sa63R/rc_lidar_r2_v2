# Rc 4.2 TF Tree Rewrite And Odom-Based Robot Pose Source

## Summary

这篇日志记录一次关键大改：先把机器人业务 TF 树重写成无头车语义，再发现直接从 TF buffer 查询 `robot_origin -> robot_base_highrate` 会放大抖动，最后改为保留新 TF 树、但生产 `/robot/pose` 改回基于 `/map_lidar/odom` 的同帧数学计算。

当前对应 commit：

```text
d8b3b3a Rc 4.2: restore odom-based robot pose source
```

这次改动的目标不是删除 TF，而是区分两件事：

- TF 树：表达物理帧关系，用于 RQT、tf2_echo、外参校准和调试。
- 生产坐标：发给 PlotJuggler、滤波、CAN 的 `/robot/pose`，优先用同一帧定位结果计算，避免混合不同频率 TF 造成抖动。

## Background

当前业务坐标约定：

```text
X = 机器人右方
Y = 地图正方向 / 机器人前方
Z = 上方
yaw = 左转为正
```

雷达和机器人中心的已校准外参继续沿用：

```text
x = -0.240
y =  0.287302
z = -0.760
roll  = 0
pitch = 0
yaw   = +1.5707963267948966
```

用户希望做无头车，所以地图正方向必须稳定作为业务世界的 `Y+`。因此新增或明确了 `robot_origin`：它是机器人业务原点，静态固定在地图中，和 PCD 地图原点一样属于建图/启动时定死的参考。

## First Design: New TF Tree

第一步把旧的 `world/init/base_link` 语义改成更清楚的无头车 TF：

```text
map
├── camera_init
│   ├── body -> robot_base
│   └── body_highrate -> robot_base_highrate
└── robot_origin
```

各帧含义：

- `map`：PCD 地图坐标。Y 正方向就是无头车世界正方向。
- `camera_init`：FAST-LIO 里程计初始帧。重定位时由 `map_localizer` 发布 `map -> camera_init`。
- `body`：雷达当前低频帧。
- `body_highrate`：雷达当前高频帧。
- `robot_origin`：机器人业务原点，静态固定在 `map` 下。
- `robot_base`：机器人当前中心低频帧。
- `robot_base_highrate`：机器人当前中心高频帧。

这棵树本身是合理的。它保留了“地图坐标”和“机器人业务原点”的区别，也让 `robot_origin -> robot_base_highrate` 看起来就是电控关心的位姿。

## Problem Found

实测发现，大改 TF 后 `/robot/pose_filtered` 的静止波动明显变大，有时达到厘米级甚至更高。用户观察到：

- 重写 TF 之前，如果启动位置准，滤波后可以接近 5 mm 稳定。
- 重写 TF 之后，即使抵住墙边获得较准的启动角度和位置，滤波后仍可能出现约 3 cm 抖动。
- CPU 负载确实可能影响定位，但这次更可疑的是坐标来源方式变化。

RQT 里看到的 TF 频率类似：

```text
map -> camera_init                  about 7 Hz
camera_init -> body                 about 11 Hz
camera_init -> body_highrate        about 50 Hz
body -> robot_base                  static
body_highrate -> robot_base_highrate static
map -> robot_origin                 static
```

如果生产 `/robot/pose` 直接查询：

```bash
ros2 run tf2_ros tf2_echo robot_origin robot_base_highrate
```

它会把低频地图修正、高频雷达姿态和静态外参通过 TF buffer 拼起来。这个过程可能发生时间戳对齐、插值、外推或使用最近可用 transform 的问题。对于普通可视化这通常还能接受，但对于毫米到厘米级稳定度目标，会把小误差放大成明显抖动。

## Key Insight

旧版本更稳的关键不是旧 TF 树名字更好，而是生产坐标更接近“同一帧数学计算”：

```text
map-localizer 的 odom/pose 输出
  + 雷达到机器人中心外参
  + 机器人业务原点参考
  -> /robot/pose
```

也就是说，TF 树可以保留用于表达关系，但生产 `/robot/pose` 不应该依赖跨频率 TF 查询拼接。

`/map_lidar/odom` 的含义：

- 它不是电控最终坐标。
- 它是地图重定位节点输出的“雷达/车体在 map 下的定位结果”。
- 它已经包含地图匹配后的定位修正。
- 用它作为输入，再加雷达到机器人中心外参，可以得到机器人中心在地图下的位置。

最终生产位姿应按这个公式算：

```text
robot_origin -> robot_base =
inverse(map -> robot_origin) * (map -> body) * (body -> robot_base)
```

这个公式的优势是所有关键量都在同一次 `/map_lidar/odom` 回调里计算，不需要 TF buffer 同步不同频率的动态边。

## Final Design

保留新 TF 树，但把重定位生产 `/robot/pose` 改为 `odom` 来源：

```text
/map_lidar/odom
  -> pose_from_odom
  -> /robot/pose
  -> pose_stability_filter
  -> /robot/pose_filtered
  -> CAN_SERIAL
```

默认启动参数：

```bash
RC_ROBOT_POSE_SOURCE=odom
```

临时回退旧 TF 拼接路径：

```bash
RC_ROBOT_POSE_SOURCE=tf ./autostart/start_all_terminals.sh
```

回退路径只用于对比和诊断，不建议作为生产调车默认链路。

## Files And Impact

### Startup Scripts

`autostart/pose_interface_relocalization.sh` 默认改为：

```bash
RC_ROBOT_POSE_SOURCE=odom
```

`odom` 模式启动：

```bash
ros2 launch lidar_localization lidar_pose_from_odom.launch.py \
  input_odom_topic:=/map_lidar/odom \
  zero_on_first_pose:=false \
  zero_frame_id:=robot_origin \
  reference_pose_enabled:=true \
  reference_frame_id:=robot_origin
```

`tf` 模式仍存在，启动 `pose_from_tf` 查询 `robot_origin -> robot_base_highrate`。

`start_all_terminals.sh` 不需要改变整体顺序，仍是：

```text
Livox/FAST-LIO/map_localizer
-> POSE_INTERFACE
-> pose filters
-> ready notify / health monitor / CAN / PlotJuggler
```

### TF Tree

TF 树继续保留：

```text
map -> camera_init
camera_init -> body
camera_init -> body_highrate
map -> robot_origin
body -> robot_base
body_highrate -> robot_base_highrate
```

它现在主要用于：

- RQT 观察物理关系。
- `tf2_echo` 手动检查。
- 自转外参校准。
- `/tf_compare/...` 调试话题。

生产 `/robot/pose` 默认不再由 TF 查询直接得到。

### PlotJuggler

PlotJuggler 订阅的话题名不需要变：

```text
/robot/pose
/robot/pose_filtered
/robot/pose_filtered_stable
/robot/pose_filtered_beta
/map_lidar/quality
/lidar/pose_filter/status
/tf_compare/robot_origin_to_robot_base
/tf_compare/robot_origin_to_robot_base_highrate
/tf_compare/map_to_body
```

变化在于：

- `/robot/pose` 现在是 odom 同帧计算得到的机器人业务坐标。
- `/tf_compare/...` 仍是 TF 路径对比，主要用于判断 TF 查询路径是否比 odom 路径更抖。
- 如果 PlotJuggler 里只看 `/tf_compare/robot_origin_to_robot_base_highrate`，看到的仍然是 TF buffer 拼接结果，不代表 CAN 默认发送值。

### CAN

CAN 帧格式不变：

```text
0x12: float x, float y
0x13: float z, float yaw
```

重定位模式默认仍是：

```text
RC_CAN_POSE_SOURCE=pose_topic
RC_CAN_POSE_TOPIC=/robot/pose_filtered
```

因此电控收到的是：

```text
/map_lidar/odom
-> pose_from_odom
-> /robot/pose
-> normal/stable/beta filter
-> /robot/pose_filtered
-> CAN
```

如果 `/robot/pose_filtered` 不存在或不刷新，CAN 会暂停发送，不会悄悄退回 TF 链路，避免坐标来源混乱。

### Health Logs

健康日志字段不需要重命名，但含义要注意：

- raw pose `/robot/pose`：现在是 odom-derived 机器人业务位姿。
- filtered pose `/robot/pose_filtered`：滤波后的机器人业务位姿。
- map quality `/map_lidar/quality`：地图匹配质量。
- tf compare topics：TF 路径对比，不是默认 CAN 源。

### Ready Notify

ready 逻辑仍观察：

```text
/robot/pose_filtered
/map_lidar/quality
```

因为话题名没变，所以无需额外改动。

### Stop And Restart

`stop_all.sh` 需要确保同时清理：

```text
pose_from_odom
pose_from_tf
pose_stability_filter
```

CAN 自动重启和开机自启默认继承 `RC_ROBOT_POSE_SOURCE=odom`。只有显式设置环境变量时，才会走 `tf` 回退模式。

### Mapping Mode

这次主要修复重定位生产链路。mapping 模式的核心仍是 FAST-LIO 建图和保存 PCD。迁移到其他仓库时，不要把“重定位 pose 来源修复”误解成必须改变建图保存逻辑。

## Migration Notes For Other Repositories

迁移到 r2 或其他仓库时，不要机械复制所有文件。先确认这些事实：

1. 是否有地图重定位输出，例如 `/map_lidar/odom`。
2. 这个 odom 的 child/body 是否代表雷达当前帧，还是机器人中心。
3. 雷达到机器人中心的外参是否已知。
4. 业务世界原点是不是需要 `robot_origin`。
5. CAN 期待的坐标系是不是 `X右/Y前/Z上`。
6. PlotJuggler 当前订阅的是 pose topic 还是 tf_compare topic。

推荐迁移顺序：

```text
先保留 TF 树
-> 新增 odom-derived /robot/pose
-> PlotJuggler 同时观察 /robot/pose 与 /tf_compare
-> 确认 /robot/pose 更稳后
-> CAN 默认切到 /robot/pose_filtered
-> TF 路径仅作为诊断/回退
```

不要一开始就删除 TF 树。TF 树仍然对校准、调试和理解物理关系很有价值。

## Verification Commands

启动：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=1 \
RC_POSE_FILTER_STABLE=1 \
RC_LIDAR_MAP_PATH="/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260625_064655.pcd" \
./autostart/start_all_terminals.sh
```

检查 pose 接口：

```bash
tail -n 80 logs/autostart/latest/POSE_INTERFACE.log
ros2 topic echo /map_lidar/odom --once
ros2 topic echo /robot/pose --once
ros2 topic echo /robot/pose_filtered --once
```

检查 CAN：

```bash
tail -n 120 logs/autostart/latest/CAN_SERIAL.log
ip -details -statistics link show can0
```

检查 TF 对比：

```bash
ros2 run tf2_ros tf2_echo robot_origin robot_base_highrate
ros2 topic echo /tf_compare/robot_origin_to_robot_base_highrate --once
```

如果 `/robot/pose_filtered` 比 `/tf_compare/robot_origin_to_robot_base_highrate` 稳，说明这次修复方向正确。

## Lessons Learned

- TF 树结构正确，不代表“用 TF buffer 查询出来的生产坐标”一定最稳。
- 高频和低频动态 TF 混合时，厘米级精度目标会暴露时间戳对齐问题。
- PlotJuggler 里必须分清楚：
  - `/robot/pose`：生产原始位姿。
  - `/robot/pose_filtered`：生产滤波位姿，默认 CAN 源。
  - `/tf_compare/...`：TF 路径诊断值。
- CAN 坐标来源必须单一明确；不要让 CAN 在 topic 不存在时自动退回旧 TF，否则调试时很难判断电控到底收到哪条链路。
- Orange Pi 上用 Codex 调试时通常会开 VS Code 和 Clash，这会显著增加 CPU 负载。判断滤波或定位质量时，应尽量使用 headless 低负载日志做最终依据。
