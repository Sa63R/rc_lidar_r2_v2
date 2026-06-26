可以实现，但我不想说“百分百一次就完美”，因为这里有两个现场相关风险：时间戳是否可信、`/map_lidar/odom` 到底代表 `map -> body` 还是已经带了某些业务外参。我的计划会先做成**并行新链路**，不直接替换现有 CAN，先让你和 GPT 都能审阅、验证，再一键切换。

**目标**
新增一条连续位姿链路：

```text
/Odometry 高频连续 FAST-LIO 位姿
/map_lidar/odom 低频地图重定位修正
body -> robot_base 外参

=> /robot/pose_continuous
```

核心公式：

```text
map_T_camera_init = map_T_body_from_localizer * inverse(camera_init_T_body_from_odom)

map_T_body_continuous = map_T_camera_init_latest * camera_init_T_body_current

robot_origin_T_robot_base_continuous =
  inverse(map_T_robot_origin)
  * map_T_body_continuous
  * body_T_robot_base
```

**计划**
1. 新增节点，不动旧链路  
   新建类似：

```text
src/lidar_localization/lidar_localization/continuous_robot_pose.py
```

订阅：

```text
/Odometry
/map_lidar/odom
/map_lidar/quality
```

发布：

```text
/robot/pose_continuous
/robot/pose_continuous/status
```

先不替换 `/robot/pose`，也不立刻改 CAN，避免一上来影响当前可用版本。

2. 节点内部维护 odom 历史缓存  
   因为 map localizer 结果是低频的，必须尽量找到“同一时刻”的 FAST-LIO odom：

```text
odom_buffer: 保存最近 3-5 秒 /Odometry
map_lidar_odom 到来时：
  按 header stamp 找最近 odom
  如果 stamp 不可信，则 fallback 到最近收到的 odom
```

状态里记录：

```text
matched_odom_age
stamp_delta
used_exact_time_match
used_latest_fallback
```

这样我们能判断它是不是时间对齐出了问题。

3. 每次 map localizer 更新时，只更新 correction  
   map localizer 到来时不直接发最终 pose，而是算：

```text
candidate_map_T_camera_init =
  map_T_body_measured * inverse(camera_init_T_body_at_same_time)
```

然后根据质量决定是否接受：

```text
/map_lidar/quality accepted/converged
fitness 是否达标
map jump 是否太大
candidate correction 是否离谱
```

质量差就：

```text
不更新 map_T_camera_init
继续用旧 correction + 当前 /Odometry 连续输出
```

这点很重要：quality 不好时不冻结最终 pose，只是不更新地图修正。

4. correction 不硬跳，默认平滑修正  
   增加参数：

```bash
RC_CONTINUOUS_POSE_CORRECTION_ALPHA=0.05
```

每次 map localizer 给出新 correction，不直接替换，而是：

```text
translation: 线性插值
rotation: 四元数 slerp 或只对 yaw 做角度插值
```

默认先保守：

```text
XY / yaw 修正
Z / roll / pitch 可以先保留或低权重修正
```

这样避免 map localizer 单帧噪声把连续 pose 拉跳。

5. 每次 `/Odometry` 到来时发布连续 pose  
   这一步是核心：

```text
map_T_body_continuous =
  map_T_camera_init_latest * current_camera_init_T_body
```

然后加你当前外参：

```text
body_T_robot_base:
  x=-0.240
  y=0.287302
  z=-0.760
  yaw=+pi/2
```

再转到机器人业务原点：

```text
robot_origin_T_robot_base_continuous =
  inverse(map_T_robot_origin) * map_T_body_continuous * body_T_robot_base
```

发布 `/robot/pose_continuous`。

6. CAN 先不默认切过去  
   新增参数：

```bash
RC_CAN_POSE_TOPIC=/robot/pose_continuous
```

但默认暂时仍可以保持现状，或者我们做成：

```bash
RC_CAN_POSE_SELECTED=filtered|continuous
```

你测试确认后再切默认。CAN yaw 继续按你现在要求：

```text
0x13 yaw = 原始 /Odometry yaw
```

也就是说这次主要改 XY/Z 的连续性，不碰 raw odom yaw 的方向。

7. PlotJuggler 增加对比话题  
   自动选中：

```text
/robot/pose/pose/position/x,y
/robot/pose_continuous/pose/position/x,y
/robot/pose_filtered/pose/position/x,y
/robot/pose_odom_only/pose/position/x,y
/map_lidar/odom/pose/pose/position/x,y
/robot/pose_continuous/status
```

这样你能直接看到：

```text
map localizer 阶梯
纯 odom 连续但漂
continuous 连续且被地图慢慢修正
filtered 是否还有必要
```

8. 健康日志加入关键字段  
   增加记录：

```text
continuous_pose_x/y/yaw
correction_x/y/yaw
correction_alpha
correction_age
matched_odom_age
map_update_accepted
map_update_rejected_reason
continuous_vs_map_error
continuous_vs_odom_error
```

这样之后能判断：

```text
是 map 修得太慢
还是 odom 漂得太快
还是时间戳没对齐
还是 correction 被质量门控挡住
```

9. 启动脚本接入但默认安全  
   新增：

```text
autostart/continuous_robot_pose.sh
```

在 relocalization 模式启动。初期可以默认启动但不接 CAN，因为它只是多发布一个 topic，风险小。

新增环境变量：

```bash
RC_CONTINUOUS_POSE=1
RC_CONTINUOUS_POSE_INPUT_ODOM=/Odometry
RC_CONTINUOUS_POSE_INPUT_MAP_ODOM=/map_lidar/odom
RC_CONTINUOUS_POSE_OUTPUT=/robot/pose_continuous
RC_CONTINUOUS_POSE_CORRECTION_ALPHA=0.05
RC_CONTINUOUS_POSE_USE_QUALITY=1
```

10. 分阶段验收  
   第一阶段只看 topic：

```text
/robot/pose_continuous 是否连续
是否比 /robot/pose 少阶梯
静止误差是否仍在 5mm-1cm 级别
```

第二阶段手动切 CAN：

```bash
RC_CAN_POSE_TOPIC=/robot/pose_continuous
```

第三阶段如果效果好，再改默认开机启动。

**我最担心的点**
1. `/map_lidar/odom` 的 frame 语义  
   必须确认它确实是 `map_T_body` 或等价雷达当前位姿。如果它已经带了 robot 外参，公式要改。

2. 时间戳  
   如果 `/map_lidar/odom` 的 stamp 和 `/Odometry` 不在同一时间基准，需要用接收时间或 map localizer 自己记录的 matched odom。

3. correction 平滑力度  
   `alpha=0.05` 可能稳但修正慢；`alpha=0.2` 修得快但可能抖。这个要实测。

**我会怎么保证不改崩**
先不替换 `/robot/pose`，先不默认切 CAN，只新增 `/robot/pose_continuous`。  
旧链路保留：

```text
/robot/pose
/robot/pose_filtered
/robot/pose_odom_only
CAN raw odom yaw
```

所以即使新链路效果不好，也能一键不用它。

这套实现本质就是把你现在的系统变成：

```text
map -> camera_init -> body -> robot_base
```

其中：

```text
camera_init -> body: FAST-LIO 连续 odom
map -> camera_init: map localizer 低频修正
```

这正是我们讨论的 FAST-LIO-LOCALIZATION 思路。