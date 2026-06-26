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

后续增加一个只随 PlotJuggler 调试启动的对比 topic：

```text
/Odometry
  -> pose_from_odom 加同一组 body -> robot_base 外参
  -> /robot/pose_odom_only
```

`/robot/pose_odom_only` 不经过 map localizer、不进入滤波、不发给 CAN，只用于和 `/robot/pose`、`/robot/pose_filtered` 在 PlotJuggler 里对比。它的目的，是判断“纯 FAST-LIO odom 自己已经漂/抖”，还是“map localizer 修正链路导致拉动/延迟”。默认 `RC_ODOM_ONLY_COMPARE=auto`：开机低负载不启动，手动打开 PlotJuggler 时启动。

默认启动参数：

```bash
RC_ROBOT_POSE_SOURCE=odom
```

临时回退旧 TF 拼接路径：

```bash
RC_ROBOT_POSE_SOURCE=tf ./autostart/start_all_terminals.sh
```

回退路径只用于对比和诊断，不建议作为生产调车默认链路。

## Rc 4.8 Low-Latency Yaw Fusion

后续实车观察发现，Rc 4.2 的 odom-derived `/robot/pose` 已经把静态 XY 精度恢复到接近旧版本，但无头车自转时仍有明显 yaw 延迟。进一步排查后确认：

- `RC_POSE_FILTER_USE_RAW_YAW=1` 已经让 `/robot/pose_filtered` 的 yaw 直接取 `/robot/pose`，滤波门控不再是主要延迟来源。
- 但 `/robot/pose` 的 yaw 来自 `/map_lidar/odom`，而 `/map_lidar/odom` 是地图重定位后的位姿，不是 FAST-LIO 最原始的高频 yaw。
- 健康日志里 `map_quality_scan_age_sec`、`map_quality_odom_age_sec` 和 `map_quality_align_ms` 可以解释一部分 map-localizer 链路延迟。

因此新增低延迟 yaw 融合，不改变 XY 来源：

```text
XY:
  /map_lidar/odom
  -> pose_from_odom
  -> robot_origin-relative x/y/z

yaw:
  latest map yaw reference
  + FAST-LIO /Odometry yaw delta since that reference
```

核心公式：

```text
fused_yaw =
  yaw_map_at_last_good_update
  + wrap(yaw_fastlio_now - yaw_fastlio_at_last_good_update)
```

每次新的 `/map_lidar/odom` 到来时，不是直接硬跳到新的 map yaw，而是按 `RC_ROBOT_POSE_YAW_FUSION_MAP_CORRECTION_ALPHA` 做小步全局校正；两次 map 更新之间，则由 FAST-LIO `/Odometry` 回调快速发布新 yaw。这样：

- 地图重定位继续负责全局方向，不让 yaw 长期漂掉。
- FAST-LIO 高频 yaw 负责短时间转向响应。
- `/robot/pose` 的 XY 仍保持地图重定位结果，不引入 FAST-LIO 纯里程计平移漂移。

新增默认参数：

```bash
RC_ROBOT_POSE_YAW_FUSION=1
RC_ROBOT_POSE_FAST_ODOM_TOPIC=/Odometry
RC_ROBOT_POSE_YAW_FUSION_PUBLISH_ON_FAST_ODOM=1
RC_ROBOT_POSE_YAW_FUSION_MAX_FAST_AGE=0.50
RC_ROBOT_POSE_YAW_FUSION_MAP_CORRECTION_ALPHA=0.05
```

回退旧 map yaw：

```bash
RC_ROBOT_POSE_YAW_FUSION=0 ./autostart/start_all_terminals.sh
```

新增诊断 topic：

```text
/robot/yaw_fusion/status
```

字段含义：

```text
data[0] enabled
data[1] ready
data[2] source_code: 1=map update, 2=fast odom update
data[3] fast_age_sec
data[4] map_age_sec
data[5] map_ref_yaw_rad
data[6] fast_yaw_rad
data[7] fused_yaw_rad
data[8] fast_delta_yaw_rad
data[9] map_correction_alpha
```

PlotJuggler 和健康日志都同步加入 `/robot/yaw_fusion/status`。后续判断 yaw 延迟时，优先看：

- `yaw_fusion_ready` 是否为 1。
- `yaw_fusion_fast_age_sec` 是否长期很小。
- `yaw_fusion_map_age_sec` 是否远大于 fast age。
- `/robot/pose` 和 `/robot/pose_filtered` 的 yaw 是否仍有明显延迟。

## Rc 4.9 CAN Uses Fused Yaw Directly

Rc 4.8 后，`/robot/pose` 的 yaw 已经是地图 yaw 基准加 FAST-LIO 高频 yaw 增量；但 CAN 仍默认整帧读取 `/robot/pose_filtered`。实车现象是：即使 `/robot/pose` 的 yaw 响应明显快了，电控拿到的 yaw 仍可能被滤波链路拖慢。

因此 Rc 4.9 把 CAN 的位置和 yaw 来源拆开：

```text
CAN x/y/z:
  /robot/pose_filtered

CAN yaw:
  /robot/pose
```

也就是：

```bash
RC_CAN_POSE_SOURCE=pose_topic
RC_CAN_POSE_TOPIC=/robot/pose_filtered
RC_CAN_USE_YAW_POSE_TOPIC=1
RC_CAN_YAW_POSE_TOPIC=/robot/pose
RC_CAN_YAW_POSE_TOPIC_STRICT=1
```

这样电控仍然拿到滤波后的稳定位置，但航向角直接来自融合 yaw，避免无头车自转时继续受 `/robot/pose_filtered` 的 yaw 延迟影响。

`RC_CAN_YAW_POSE_TOPIC_STRICT=1` 的含义是：如果 `/robot/pose` 的 yaw topic 不存在或超时，CAN 直接暂停发送，而不是悄悄回退到 `/robot/pose_filtered` 的慢 yaw。这样调车时来源更明确，出问题也更容易从 `CAN_SERIAL.log` 里看出来。

回退旧行为：

```bash
RC_CAN_USE_YAW_POSE_TOPIC=0 ./autostart/start_all_terminals.sh
```

这次修改不改变 CAN 帧格式，仍然是：

```text
0x12: float x, float y
0x13: float z, float yaw
```

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
/robot/pose_odom_only
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

重定位模式默认位置来源仍是：

```text
RC_CAN_POSE_SOURCE=pose_topic
RC_CAN_POSE_TOPIC=/robot/pose_filtered
RC_CAN_USE_YAW_POSE_TOPIC=1
RC_CAN_YAW_POSE_TOPIC=/robot/pose
```

因此电控收到的是：

```text
/map_lidar/odom
-> pose_from_odom
-> /robot/pose
-> normal/stable/beta filter
-> /robot/pose_filtered
-> CAN x/y/z

/robot/pose
-> CAN yaw
```

如果 `/robot/pose_filtered` 不存在或不刷新，CAN 会暂停发送，不会悄悄退回 TF 链路，避免坐标来源混乱。Rc 4.9 起，如果 `/robot/pose` 的 yaw 不存在或不刷新，默认也会暂停发送。

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

## Rc 4.10: CAN Yaw Bypasses Ready Gate

实车继续排查无头车旋转延迟时，发现即使 CAN 的 yaw 已经单独读取 `/robot/pose`，它仍然会被 `RC_CAN_REQUIRE_READY=1` 的 `/map_lidar/quality` ready gate 一起冻结。也就是说，只要 map localizer 短暂不 ready，旧逻辑会整包 `return`，`0x12` 和 `0x13` 都不发。对电控来说，这等价于 yaw 短时间停在旧值，表现为车已经转了，但无头车逻辑还不知道朝向已经变了。

Rc 4.10 把 CAN ready gate 拆开：

```text
0x12: x/y
  继续受 /map_lidar/quality ready gate 保护。
  quality 不 ready 时不发送新的 x/y。

0x13: z/yaw
  yaw 默认不受 ready gate 冻结。
  quality 不 ready 时，继续发送 /robot/pose 的最新 yaw。
  z 使用最近一次 quality 通过时保存的可信 z。
```

新增默认开关：

```bash
RC_CAN_YAW_IGNORE_READY_GATE=1
```

这个开关只绕过 CAN ready gate 对 yaw 的冻结，不关闭 `/robot/pose` 新鲜度检查。`RC_CAN_YAW_POSE_TOPIC_STRICT=1` 仍然有效：如果 `/robot/pose` 不存在或过期，CAN 仍会暂停，避免发送未知来源 yaw。

如果现场想恢复旧行为，可以临时设置：

```bash
RC_CAN_YAW_IGNORE_READY_GATE=0
```

这个改动不是为了解决所有旋转延迟，而是为了排除一个明确风险：地图质量短暂波动时不应该让无头车航向角完全冻结。x/y 继续走 ready gate，避免低质量定位结果污染位置控制。

## Rc 4.18: Default CAN Ready Gate Disabled

后续继续排查 map localizer 离散阶梯输出和电控无头车响应时，用户要求先把默认启动里的 CAN ready gate 关掉。这次只改默认值，不删除门控功能：

```bash
RC_CAN_REQUIRE_READY=0
```

影响：

- CAN 默认不再等待 `/map_lidar/quality` 连续 accepted/converged 才发送。
- `x/y/z` 仍来自当前 `RC_CAN_POSE_TOPIC`，默认 `/robot/pose_filtered`。
- `yaw` 仍按当前 raw FAST-LIO `/Odometry` 策略发送。
- 如果需要恢复旧保护，启动时设置 `RC_CAN_REQUIRE_READY=1` 即可。

这次改动的目的不是改变坐标算法，而是排除 quality ready gate 对电控链路造成冻结或误判的影响。真正的定位质量仍然需要通过 `/map_lidar/quality`、健康日志和 PlotJuggler 观察。

## Rc 4.11: map localizer drops queued history instead of replaying old scan/odom

后续实车观察发现，即使 CAN yaw 已经直接读取 `/robot/pose`，无头车旋转时仍存在“车停了，yaw 还继续慢慢追到正确角度”的现象。通过 `/robot/yaw_fusion/status` 对比可见：

- `fast_yaw` 基本实时，停住后只在很小范围内抖动。
- `map_ref/fused_yaw` 会在 `map_update` 中继续缓慢变化，像是在把旧 map 修正结果一帧帧拉回来。
- 健康日志里 `map_quality_scan_age_sec`、`map_quality_odom_age_sec` 和 `align_ms` 可以用来判断 map localizer 是否真的在处理旧输入或算得太慢。

这说明问题不只是 `alpha` 大小，而是 map localizer 不应该让旧 scan/odom 排队进入实时控制修正链路。Rc 4.11 做了两件保守修改：

1. `map_localizer` 的 scan 和 odom 输入 QoS 改成 `keep_last(1)`，并通过 `RC_MAP_LOCALIZER_INPUT_QUEUE_DEPTH` 暴露配置，默认 `1`。如果地图匹配耗时高于输入频率，旧输入会被最新输入覆盖，而不是排队补算历史。
2. `RC_ROBOT_POSE_YAW_FUSION_MAP_CORRECTION_ALPHA` 默认从 `0.25` 降到 `0.05`。FAST-LIO yaw 继续负责快速响应，map yaw 只做较弱的全局方向修正，降低旧 map 结果强拖 yaw 的概率。

这个版本暂时不做“按时间戳强丢弃”，因为 Livox/FAST-LIO/header stamp 和系统 wall time 可能不是完全相同的时间基准。更稳的策略是先防止 DDS/ROS 队列积压历史输入，同时继续把 `scan_age_sec`、`odom_age_sec`、`align_ms` 写进健康日志。如果后续证明时间戳可信，再考虑加入“过旧结果只记录、不参与 yaw correction”的第二层门控。

迁移到其他仓库时需要同步：

```bash
RC_MAP_LOCALIZER_INPUT_QUEUE_DEPTH=1
RC_ROBOT_POSE_YAW_FUSION_MAP_CORRECTION_ALPHA=0.05
```

注意：这不会让 map localizer “完全不慢”，它只是避免旧输入排队回放。若单次匹配本身仍要 150ms 以上，map 修正仍天然晚一个计算周期；但它不应再累积成 1-2 秒历史回放。

## Rc 4.12: CAN yaw uses raw FAST-LIO odom yaw

Rc 4.11 后，用户实测认为“只保留最新 map 输入 + 低 map correction alpha”已经比之前好很多，但无头车仍不能完全边旋转边前进。进一步分析后，用户从电控侧得到新的临时要求：先不要让 CAN yaw 走 map localizer、yaw fusion 或 pose filter，直接发送 FAST-LIO 原始 `/Odometry` 的 yaw，用来验证最实时的航向角是否能解决无头车旋转延迟。

因此 Rc 4.12 只修改 CAN 的 yaw 来源，不改变 x/y/z：

```text
CAN x/y/z:
  /robot/pose_filtered
  继续走地图重定位、滤波和 ready gate

CAN yaw:
  /Odometry 原始 FAST-LIO yaw + pi/2
  不走 map localizer 修正
  不走 /robot/pose yaw fusion
  不走 pose filter
  不受 /map_lidar/quality ready gate 冻结
```

新增默认参数：

```bash
RC_CAN_USE_RAW_ODOM_YAW=1
RC_CAN_RAW_ODOM_YAW_TOPIC=/Odometry
RC_CAN_RAW_ODOM_YAW_TIMEOUT_SEC=0.5
RC_CAN_RAW_ODOM_YAW_STRICT=1
RC_CAN_RAW_ODOM_YAW_OFFSET_RAD=1.5707963267948966
RC_CAN_RAW_ODOM_YAW_IGNORE_READY_GATE=1
```

`+pi/2` 是业务坐标偏置，用来把 FAST-LIO/LiDAR 原始 yaw 转到当前无头车 `X右/Y前/Z上` 的 yaw 约定。输出仍 wrap 到 `[-pi, pi]`，符合电控原本期待。

回退方式：

```bash
RC_CAN_USE_RAW_ODOM_YAW=0
```

`RC_CAN_RAW_ODOM_YAW_STRICT=1` 表示 raw `/Odometry` yaw 不存在或超时时暂停发送，不悄悄 fallback 到 `/robot/pose` 融合 yaw，避免现场误判当前到底发给电控的是哪条 yaw 链路。

关闭后 CAN yaw 会回到 Rc 4.9 的 `/robot/pose` 低延迟融合 yaw。若还要完全退回“整包用 pose topic 的 yaw”，再设置 `RC_CAN_USE_YAW_POSE_TOPIC=0`。

风险记录：

- 原始 `/Odometry` yaw 实时性最高，但转多圈后可能存在 FAST-LIO/IMU 累计漂移。
- 该改动只用于验证电控无头车实时转向；如果漂移影响比赛，需要回到“FAST-LIO 高频预测 + map 慢速偏置修正”的延迟补偿方案。
- x/y 仍不能绕过 ready gate，否则地图匹配不稳定时会把错误位置发给电控。

## Rc 4.13: raw odom CAN yaw offset changed to pi

Rc 4.12 默认使用 raw `/Odometry` yaw 加 `+pi/2`。现场继续观察后，用户判断“ODOM 的 Y 指向地图右方，但业务 Y 需要指向车前方”，因此要求在当前基础上再加 `90°` 试车。Rc 4.13 只修改 raw odom CAN yaw 的默认偏置：

```bash
RC_CAN_RAW_ODOM_YAW_OFFSET_RAD=3.141592653589793
```

也就是说：

```text
yaw_can = wrap(yaw_from_/Odometry + pi)
```

其它链路保持不变：

- CAN x/y/z 仍来自 `/robot/pose_filtered`。
- raw odom yaw 仍默认 `STRICT=1`，不新鲜就暂停，不 fallback。
- map yaw fusion、pose filter、TF 树和外参不变。

如果现场验证发现方向反了，可以不改代码直接覆盖：

```bash
RC_CAN_RAW_ODOM_YAW_OFFSET_RAD=1.5707963267948966
```

或回退到融合 yaw：

```bash
RC_CAN_USE_RAW_ODOM_YAW=0
```

## Rc 4.14: raw odom CAN yaw offset reset to zero

继续梳理后，用户意识到：如果电控需要的是“原始 odom yaw 的旋转角”，并且 `/Odometry` 与电控都按逆时针为正，那么雷达轴和车体轴之间的固定夹角不一定需要作为 yaw offset 加到 CAN。对 yaw 来说，刚体上任意一根固定轴的旋转增量相同；如果电控只关心旋转角/偏角，最直接的验证方式就是发原始 yaw。

因此 Rc 4.14 把 raw odom CAN yaw 偏置改回 0：

```bash
RC_CAN_RAW_ODOM_YAW_OFFSET_RAD=0.0
```

当前 CAN yaw 公式为：

```text
yaw_can = wrap(yaw_from_/Odometry)
```

保留参数是为了现场继续试错。如果之后确认电控需要绝对零点偏置，不改代码也可以覆盖：

```bash
RC_CAN_RAW_ODOM_YAW_OFFSET_RAD=1.5707963267948966
# 或
RC_CAN_RAW_ODOM_YAW_OFFSET_RAD=3.141592653589793
```

## Rc 4.15: boot autostart fixed to low-load raw-odom-yaw relocalization

用户要求把当前可用启动命令设置为开机启动：

```bash
cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
RC_LIDAR_MAP_PATH="/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260625_064655.pcd" \
./autostart/start_all_terminals.sh
```

因此开机入口 `boot_start.sh` 和 user systemd 模板 `rc-lidar-autostart.service` 明确固定为：

```text
RC_LIDAR_MODE=relocalization
RC_LIDAR_MAP_PATH=/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260625_064655.pcd
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1
RC_AUTOSTART_HEADLESS=1
RC_AUTOSTART_PLOTJUGGLER=0
RC_CAN_POSE_SOURCE=pose_topic
RC_CAN_POSE_TOPIC=/robot/pose_filtered
RC_CAN_USE_YAW_POSE_TOPIC=0
RC_CAN_USE_RAW_ODOM_YAW=1
RC_CAN_RAW_ODOM_YAW_TOPIC=/Odometry
RC_CAN_RAW_ODOM_YAW_STRICT=0
RC_CAN_RAW_ODOM_YAW_OFFSET_RAD=0.0
RC_CAN_RAW_ODOM_YAW_IGNORE_READY_GATE=1
RC_POSE_FILTER_STABLE=0
```

这表示开机和无显示器场景下，核心链路仍然 headless 后台启动；CAN 的 `x/y/z` 走 normal `/robot/pose_filtered`，`yaw` 走 FAST-LIO 原始 `/Odometry` yaw，偏置为 0。旧的 `/robot/pose` yaw topic override 不再作为开机默认，以避免读文档和日志时误判 yaw 来源。

## Rc 4.19: raw odom CAN yaw no longer strict by default

用户继续实车观察后强调：“yaw 要发原始 odom”，不能因为质量门控或超时保护又把航向角冻住。复查代码发现，虽然 CAN yaw 来源已经是 `/Odometry`，但 `RC_CAN_RAW_ODOM_YAW_STRICT=1` 时，只要 `/Odometry` 短暂超过 timeout，CAN 仍会暂停发送。这会在现场表现成 yaw 又被 hold。

因此 Rc 4.19 修改默认策略：

```bash
RC_CAN_RAW_ODOM_YAW_STRICT=0
```

并调整 `rc_serial_driver` 的 raw odom yaw 读取逻辑：

- `/Odometry` 正常更新时，继续发送最新原始 odom yaw。
- `/Odometry` 短暂 stale 时，默认继续使用最后一帧原始 odom yaw。
- 不 fallback 到 `/robot/pose_filtered` 的 yaw。
- 需要恢复“raw odom yaw 超时就暂停 CAN”的保守保护时，手动设置 `RC_CAN_RAW_ODOM_YAW_STRICT=1`。

这次只处理 CAN yaw 的 stale/strict 语义，不改变 x/y/z 来源，不改变 TF 树，不改变 map localizer。

## Rc 4.20: add continuous pose from FAST-LIO odom plus map correction

继续分析后，用户确认了一个更完整的连续定位思路：`/map_lidar/odom` 本身是 map localizer 一帧一帧算出来的离散结果，如果直接把它转换成 `/robot/pose`，曲线会带有阶梯感；而 FAST-LIO `/Odometry` 是连续局部位姿，但会有局部漂移。因此更合理的结构是：

```text
/Odometry                         # camera_init -> body，连续但会漂
/map_lidar/odom                   # map -> body，低频但给全局校正
map_T_camera_init = map_T_body * inverse(camera_init_T_body)
smoothed(map_T_camera_init)
smoothed(map_T_camera_init) * latest(camera_init_T_body)
  -> continuous map_T_body
  -> body_T_robot_base 外参
  -> robot_origin_T_robot_base
  -> /robot/pose_continuous
```

这次新增 `continuous_robot_pose` 节点，但保持保守接入：

- 默认 `RC_CONTINUOUS_POSE=auto`，只在重定位模式启动。
- 输出 `/robot/pose_continuous`、`/robot/odom_continuous`、`/robot/pose_continuous/status`。
- 不默认替换 `/robot/pose`、`/robot/pose_filtered` 或 CAN。
- PlotJuggler 自动订阅 `/robot/pose_continuous`，健康日志记录 continuous pose 和 correction 状态。
- 如果现场确认这条链路更适合控制，再手动设置 `RC_CAN_POSE_TOPIC=/robot/pose_continuous` 试验；CAN yaw 仍保持 Rc 4.19 的 raw odom yaw 策略。

状态 topic `/robot/pose_continuous/status` 记录：

```text
data[1]  correction_ready
data[2]  本次 map update 是否接受
data[3]  correction_alpha
data[4]  correction_age_sec
data[5]  odom_age_sec
data[6]  matched_stamp_delta_sec
data[7]  candidate_delta_xy_m
data[8]  candidate_delta_yaw_deg
data[11] quality_fitness
data[13] update_count
data[14] rejected_count
data[15] match_mode_code
data[19] reject_reason
```

这个节点的目的不是让 map localizer 算得更快，而是避免把低频/离散 map pose 直接作为控制位姿。它把 map localizer 的输出变成一条平滑更新的全局校正，控制输出则跟随最新 FAST-LIO odom。

## Appendix: User Questions And Constraints

以下内容按时间顺序整理本次大改相关的用户侧问题、判断和约束。重复发送的同一问题已合并；长启动命令、截图和终端日志只保留摘要。

1. 用户先要求“给当前版本上传 GitHub 打个 tag：大改前”，目的是在重写 TF 树前保存一个可恢复检查点。

2. 用户提出要重写仓库的 TF 树，并描述了核心设想：
   - 已知一定存在“雷达当前坐标”和“建图时候的雷达 init 坐标”。
   - 机械想做无头车，所以图的正向必须是 `Y`。
   - 需要额外发布一个机器人原点静态坐标，这个坐标相对于 lidar init / 地图原点固定。
   - 还需要发布一个相同外参的动态变换，从雷达当前点得到机器人当前中心。
   - 低频和高频都要有，所以预计会有 6 个 TF。
   - 滤波应该像高频坐标一样作为并列输出，而不是污染真实 TF 树。
   - 雷达前方是 `X`，左侧是 `Y`，场地正方向在雷达后方，也就是大地图 `Y` 轴；因此机器人业务坐标除了雷达到车体中心外参，还需要旋转约 `+90°`。
   - 最终发给电控的是机器人当前坐标相对机器人原点的动态坐标，以及机器人当前 `Y` 与世界 `Y` 的偏角。
   - 用户要求确认这个逻辑是否符合无头车 TF 树构建逻辑，并强调具体数值参考当前 TF，应该基本是准的。

3. 用户明确给出实施计划：“无头车 TF 树重写方案：地图 Y 正向 + 机器人业务原点”。核心要求包括：
   - `map` 表示 PCD 地图坐标，Y 正方向作为无头车世界正方向。
   - `robot_origin` 是静态业务原点。
   - `robot_base` / `robot_base_highrate` 是机器人当前中心。
   - 不要给同一个 child 发布双父节点。
   - 不额外发布 `robot_origin -> robot_base_highrate` TF 边，它应作为查询结果或 pose topic 存在。
   - 新增或改造位姿发布节点，从 TF 查询或计算机器人业务坐标，并发布 `/robot/pose`、`/robot/pose_filtered`。
   - CAN 默认继续发送 `/robot/pose_filtered`。
   - 自转外参校准工具默认帧改成 `robot_origin/body_highrate/robot_base_highrate`。

4. 大改 TF 后，用户启动 mapping 模式并报告 `save_fast_lio_map.sh` 长时间停在等待 `/map_save` 服务。这个问题提醒迁移时需要确认建图服务没有被 TF/启动脚本改动误伤。

5. 用户启动 relocalization + PlotJuggler 后报告：“PlotJuggler 未显示滤波后的坐标”。随后又补充：“PlotJuggler 里还是没看到对应话题”，并发来截图。这个问题促使检查 PlotJuggler 自动订阅是否只选中了 `/tf_compare/...`，而没有选中 `/robot/pose`、`/robot/pose_filtered`。

6. 用户报告启动一段时间后怀疑雷达程序“死机”或负载过大，但同时说明当时没有开 QQ 等额外程序。这个问题提醒不能把所有性能异常都简单归因于用户开了 GUI 软件。

7. 用户换地图后反馈：“为什么感觉现在的代码滤波后远不如之前的滤波稳定，波动又回到了 3 厘米左右，有没有可靠的办法优化，注意之前踩过很多坑，别重复踩。”这成为继续分析 TF 查询路径和生产 pose 来源的直接触发点。

8. 用户要求：“先提交上次的仓库，然后进行你刚刚计划的修改。”这要求先保存当时状态，再改动坐标来源，避免无法回退。

9. 用户要求优先跑通 CAN 链路，并说明当前滤波算法开销很大，除去 VS Code 和 Clash 也会占满 CPU；之后又明确“CPU 负载高没办法，先不管”，优先尝试指定启动命令下的 CAN 链路。

10. CAN 排查时用户说明 USB-CAN 开关已经拨回 `OR`，并确认 `FULL` 不是当前问题，因为电控接收端还没开始接收时以前也遇到过类似现象。

11. 用户确认一键启动脚本应包含 CAN 激活，并且必须免密启动；同时要求更新 MD，且每次修改后记得推送。

12. 用户反复确认开机自启链路必须稳定拉起以下 headless 低负载命令，并强调无外接显示器时也要成功启动雷达和 CAN：
    - `RC_LIDAR_MODE=relocalization`
    - `RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1`
    - `RC_AUTOSTART_HEADLESS=1`
    - `RC_AUTOSTART_PLOTJUGGLER=0`
    - `RC_POSE_FILTER_STABLE=1`
    - 指定小场可用地图 `maps/mid360_map_20260625_064655.pcd`

13. 用户提出关于滤波和 TF 的关键怀疑：
    - 重写 TF 前，如果启动位置准确，滤波后精度能维持在约 5 mm。
    - 重写 TF 后，即使用墙边抵住获得准确启动角度和位置，误差仍约 3 cm。
    - 一种猜想是滤波导致 CPU 满载，进而温度升高和定位漂移。
    - 但这个理论的问题是，旧滤波代码曾经不跑满 CPU，也能滤到 5 mm。
    - 因此更怀疑现在滤波代码写得不如之前，或者 TF 改动导致生产坐标本身变差。
    - 用户强调 Codex 调试时必须开 VS Code 和 Clash，会显著增加 CPU 负载；低功耗实车启动不会开这些，所以调试观察必须考虑这个误差。

14. 用户追问：“你的意思也就是说，滤波代码的逻辑基本上没有发生太大的变化，主要的问题就在于车的坐标开始跳动剧烈了是吗？现在滤波代码哪一个版本更加接近原来那个版本，是 Stable 版本，还是普通 filter 版本？”

15. 用户继续追问坐标源问题：
    - 当前主要原因是不是机器人坐标发布本身跳动太大？
    - 可能原因一是建图不精准。
    - 可能原因二是大改 TF 后引入误差。
    - 用户怀疑高频坐标发布可能导致问题，要求分析原来版本的 TF 是否也像现在一样 50 Hz 高频发布。
    - 如果不是高频 TF，那么最可能原因是什么？

16. 用户要求解释当前 TF 坐标变换，并特别问：“这里的 `map` 坐标系是怎么来的？我设计的 TF 变换里应该没有 `map`。”这促使文档明确：`map` 来自 PCD 地图和 map localizer，是重定位链路中的地图坐标根。

17. 用户问 Orange Pi 是否可以用 RQT 看 TF，并发来 RQT 截图。第一张图只显示 `camera_init -> body/body_highrate`，用户问是不是 RQT 自己的问题，不是 TF 真的只有这么点。随后第二张图显示完整树，用户问“现在这个 RQT 大概是完整的吗？”

18. 针对完整 RQT 图，用户追问：“你刚刚说的最可能导致误差传播的原因来源于什么，为什么之前没有这个误差传播？”这促使分析 `map -> camera_init` 低频、`body_highrate` 高频和 TF buffer 拼接可能带来的时间对齐问题。

19. 用户问：“如果想用现在这种 TF 树结构，但是实现原来那种精度，可以实现吗？另外我不太懂原来到底是怎么实现的 TF 树。”这促成最终设计：保留新 TF 树，但生产 pose 改回同帧数学计算。

20. 用户询问 `/map_lidar` 相关话题的含义：“这个话题到底是什么意思，是用来干什么的？”这促使解释 `/map_lidar/odom` 是地图重定位结果，不是电控最终坐标，但可以作为计算机器人中心 pose 的输入。

21. 用户确认理解：“TF 里算出来的雷达坐标拼起来，和直接用话题算法计算机器人中心坐标，这两者还是有差别的是吗？如果改用这种方法，计算过程就变成原来那种形式；区别只是现在机器人坐标 TF 发布语义变了，比如机身逆时针旋转 90°，但计算过程一样，是这个意思吗？”

22. 用户担心频率差：“现在 TF 坐标里面赫兹有差距，会不会因为赫兹差距导致算法计算不准？还是原来其实也有赫兹差距，但是算法算出来确实是准的？”这进一步确认生产 pose 应避免依赖跨频率 TF 查询。

23. 用户确认实现范围：“现在机器人坐标是不是直接通过 odom 话题算出来？你是不是打算直接修改 odom 话题的代码，把机器人相对雷达中心点外参加进去？之前是这么干的吗？如果我说错了就指出来。”

24. 用户进一步澄清：“低频和高频坐标不就是通过雷达做 TF 变换发布出来的吗？现在要做的也就是再加一个话题发布机器人的 pose？区别是不是不能用 TF 去查，因为高频坐标和雷达发布频率不一样，一个 50 Hz 一个 10 Hz，TF 查询机制会出现对齐问题，导致抖动非常大，所以需要手动发布话题，而不能用 TF 直接查？”

25. 用户接受方案并总结：“加了这个 pose 之后，原来的 TF 树有点冗余，但暂时不会产生明显影响。现在修改范围就是新增一个利用 odom 和雷达相对机器人中心外参，手动计算并发布机器人 pose 的话题；发布后再用这个话题减机器人原点坐标；最后把减完的坐标发给电控，是这个意思吗？”

26. 用户要求：“先确保当前仓库已提交，然后就这样改。”随后完成了 Rc 4.2 的 odom-based pose source 改动。

27. 用户最后补充要求本次修改必须全面考虑影响面，包括：
    - 启动脚本。
    - PlotJuggler 订阅话题。
    - PlotJuggler 自启动时自动订阅的话题。
    - 发给电控的 CAN 消息。
    - 以及其他所有相关链路。
   用户要求把这些修改细节都说明清楚。

28. 用户继续追问 yaw 延迟：“我现在用的 yaw 角是原始的吗？理论上原始 yaw 角 FAST-LIO 延迟是多少？电控说原始 yaw 给无头车做坐标参考基本不会有延迟，但现在依旧有一秒左右延迟。”这促使区分 `/robot/pose` 的“原始 yaw”其实来自 map-localized `/map_lidar/odom`，不是 FAST-LIO 最上游 `/Odometry`。

29. 用户问：“全局 map localization 的 yaw 是什么意思？既然 `/Odometry` 很快而且很准，是不是只要把 odom 坐标逆时针旋转 90° 再传给电控就可以？”这个问题推动说明 `/Odometry` 快但只知道本次启动局部坐标，`/map_lidar/odom` 慢但知道地图全局位置；最终方案应使用地图给全局基准、FAST-LIO 给低延迟增量。

30. 用户进一步确认：“在建图坐标系里的 `/map_lidar/odom` 是不是就是 FAST-LIO 跑出来的雷达坐标？那直接旋转 90° 是不是就可以？”这促使明确 `/map_lidar/odom` 不是最原始 FAST-LIO 输出，而是地图匹配校正后的输出；直接旋转它仍然会保留 map-localizer 延迟。

31. 用户担心融合会不会抖，并问其他仓库是否有类似策略。由此确认采用 ROS 常见的 `map` 全局校正 + `odom` 连续低延迟运动的思路，而不是在 map yaw 和 fast yaw 之间硬切换。

32. 用户要求：“之前的提交了吗，如果提交了，就按你说的进行这次修改。”于是先确认当前代码已推到 `Rc 4.7: allow raw yaw passthrough`，只剩 `note.md` 未提交，然后实施 Rc 4.8 低延迟 yaw 融合。

33. 用户要求：“直接改 CAN 的 yaw 走融合后的坐标。”这促成 Rc 4.9：CAN 的位置字段继续读 `/robot/pose_filtered`，但 yaw 字段单独读 `/robot/pose` 的融合 yaw，并默认严格要求该 topic 新鲜，避免继续把滤波延迟带给电控。

34. 用户提出新的怀疑：无头车旋转延迟可能不是 yaw topic 本身，而是 CAN ready gate 把整包输出冻结了。用户要求“yaw 不走 CAN 质量门控，其余继续走”，并且希望用参数控制，但默认 yaw 不走门控。这促成 Rc 4.10：`RC_CAN_YAW_IGNORE_READY_GATE=1` 默认开启，quality 不 ready 时只继续发 `0x13` 的 yaw，并保留 x/y 的 ready gate。

35. 用户通过 `/robot/yaw_fusion/status` 日志观察到：降低 map 修正权重到 `0.03` 后，车确实更能边转边走，但多转几圈后 FAST-LIO/odom 会有 10-20 度漂移，需要 map 慢慢拉回。用户由此判断：问题不是 fast yaw 不实时，而是 map 算得慢或使用旧点云，把旧姿态逐步拉回当前输出。

36. 用户进一步梳理 map 拖慢 fast yaw 的逻辑：`alpha=0.25` 时旧 map 结果强拉 yaw，会让机器人像被历史姿态拖住；`alpha=0.03` 时旧 map 拖不动 fast yaw，但全局修正也太慢，需要很多次 map 更新才能拉回 odom 漂移。用户要求优化这个融合逻辑，而不是继续单纯调大/调小 alpha。

37. 用户提出关键优化方向：map localizer 不应该每一个历史点云都算一遍，而应该只算当前最新点云。如果旧 scan/odom 已经没实时价值，就不要让它继续参与 yaw/pose 修正。用户特别强调：控制链路最怕旧坐标慢慢补回来。

38. 用户也指出了粗暴丢弃策略的风险：如果 map localizer 本身算得太慢，可能出现“算完发现旧了就丢，再算又旧了又丢”，最后 map 永远纠正不了 odom 漂移。因此策略不能只是按时间戳强丢弃输出。

39. 用户最终提出并确认一个更保守的第一版策略：假定 map localizer 能正常算完一帧；只维护长度为 1 的等待队列，正在算的一帧继续算，新来的点云只顶掉等待区旧帧。算完后直接用结果修正，不先做时间戳强丢弃；修正大小用参数控制，默认先从 `0.05` 试起。

40. 用户补充确认：这种只保留最新输入的改法应该也会让 XY 受益，因为 XY 同样依赖 map localizer 修正。如果旧点云参与 `/map_lidar/odom -> /robot/pose`，不仅 yaw 会被拖慢，XY 也可能被旧结果拉动。但 XY 仍应保留质量门控，不能像 yaw 一样完全绕过 ready gate。

41. 用户得到电控侧情报：MID360 自带陀螺仪，陀螺仪主要问题是温漂零偏而不是短时旋转误差，因此提出是否可以直接给电控发陀螺仪/FAST-LIO 的原始旋转数据。随后明确：原始 gyro 是角速度，不能直接当 yaw；但 FAST-LIO `/Odometry` 的 yaw 已经是 IMU/LiDAR 融合后的低延迟姿态，可以作为临时 CAN yaw 源。

42. 用户要求先保存当前仓库回退点，然后只改 CAN 的 yaw：`0x12` 的 x/y 和 `0x13` 的 z 保持现有生产链路，只有 `0x13` 的 yaw 改为 FAST-LIO 原始 `/Odometry` yaw，并叠加 `+90°` 业务坐标偏置。用户特别要求：这个 yaw 不加滤波、不走 map 重新拉回、不受 quality gate 冻结。

43. 用户确认修改范围：“只改 yaw，如果没问题，打上 tag，然后改吧。”因此本次先打 tag `before-can-raw-odom-yaw-20260626`，再实现 `RC_CAN_USE_RAW_ODOM_YAW=1` 默认启用的 raw odom yaw CAN 输出。

44. 用户随后怀疑 `+90°` 不够，并说明：“现在 ODOM 的 Y 指向地图的右方，但是我要它指向车的前方，所以理论上应该再转个 90°。”因此 Rc 4.13 把 raw odom CAN yaw 默认偏置从 `+pi/2` 改为 `+pi`，只影响 CAN yaw 字段，不改 TF 和 `/robot/pose`。

45. 用户进一步追问“旋转 Odom 的意义是什么”，并确认如果电控只需要 `/Odometry` 的原始 yaw 旋转角，且正方向一致，那么不应该额外加零点偏置。用户要求“那就改回 0”，因此 Rc 4.14 将 `RC_CAN_RAW_ODOM_YAW_OFFSET_RAD` 默认值改成 `0.0`。

46. 用户要求把当前可用低负载启动命令设置为开机启动：`RC_LIDAR_MODE=relocalization`、`RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1`、`RC_AUTOSTART_HEADLESS=1`、`RC_AUTOSTART_PLOTJUGGLER=0`、指定 `maps/mid360_map_20260625_064655.pcd`。同时要求开机只用这一套逻辑，不要多入口重复启动。因此 Rc 4.15 明确更新 `boot_start.sh` 与 user systemd 模板，并保留 boot_id guard 防止 XDG/systemd 双入口时重复真实启动。
