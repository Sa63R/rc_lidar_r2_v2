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
RC_ROBOT_POSE_YAW_FUSION_MAP_CORRECTION_ALPHA=0.25
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
