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


# 重要 0/1 开关参数速查

说明：下面这些环境变量大多数都是 `1=开启`、`0=关闭`。现场开机/电控调车优先低负载：不开 RViz、不开 PlotJuggler、不开额外调试输出；需要观察曲线时再手动打开 PlotJuggler。

推荐开机/无显示器低负载重定位：

cd "/home/orangepi/RC_LIDAR_NOW"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
RC_POSE_FILTER_STABLE=1 \
RC_LIDAR_MAP_PATH="/home/orangepi/RC_LIDAR_NOW/maps/mid360_map_20260625_064655.pcd" \
./autostart/start_all_terminals.sh

常用开关：

- `RC_AUTOSTART_HEADLESS=1`
  后台/无头启动核心节点，不给每个节点开终端。开机自启、无显示器、电控调车建议保持 `1`。

- `RC_AUTOSTART_HEADLESS=0`
  允许图形终端启动，主要用于人工调试；现场低负载不建议开。

- `RC_AUTOSTART_PLOTJUGGLER=1`
  启动 PlotJuggler，并自动订阅关键话题。调试滤波、地图质量、坐标波动时使用。

- `RC_AUTOSTART_PLOTJUGGLER=0`
  不启动 PlotJuggler。开机自启和低负载默认用这个。

- `RC_POSE_FILTER=1`
  启动普通滤波，输出 `/robot/pose_filtered`。这是默认生产滤波链路的一部分。

- `RC_POSE_FILTER=0`
  关闭普通滤波。只建议排查原始 `/robot/pose` 时临时使用。

- `RC_POSE_FILTER_STABLE=1`
  启动 stable 滤波候选，输出 `/robot/pose_filtered_stable`。当前调车方案常用这个开关打开稳定滤波能力。

- `RC_POSE_FILTER_STABLE=0`
  关闭 stable 滤波候选。用于做最低负载对照或判断滤波节点是否导致额外负载。

- `RC_POSE_FILTER_BETA=1`
  启动 beta 滤波候选，输出 `/robot/pose_filtered_beta`。只用于对比测试，默认不作为生产链路。

- `RC_POSE_FILTER_BETA=0`
  不启动 beta 滤波候选。低负载默认保持 `0`。

- `RC_CAN_REQUIRE_READY=1`
  CAN 发送前等待重定位质量达标，避免地图还没稳定就把乱坐标发给电控。生产重定位建议保持 `1`。

- `RC_CAN_REQUIRE_READY=0`
  不等待 ready，启动 CAN 后直接发。只用于排查 CAN 链路，不建议正式调车使用。

- `RC_HEALTH_MONITOR=1`
  启动健康日志，记录温度、负载、定位质量、原始/滤波位姿等。默认建议开启，开销很小。

- `RC_HEALTH_MONITOR=0`
  关闭健康日志。只有在极限压低负载时才考虑。

- `RC_READY_NOTIFY=1`
  启动重定位 ready 检测，生成 ready 标记文件；可用于判断什么时候可以动车。

- `RC_READY_NOTIFY=0`
  不启动 ready 检测。一般不建议关闭。

- `RC_READY_NOTIFY_POPUP=1`
  有显示器时弹出“重定位已就绪”的提示窗口。

- `RC_READY_NOTIFY_POPUP=0`
  不弹窗口，只写 ready 标记。无显示器/开机自启建议用 `0`。

- `RC_AUTOSTART_TF_COMPARE=1`
  启动 TF 对比话题发布器，给 PlotJuggler 看 `robot_origin -> robot_base` 等对比曲线。

- `RC_AUTOSTART_TF_COMPARE=0`
  不启动 TF 对比调试节点。低负载/不看 PlotJuggler 时建议关闭。

- `RC_AUTOSTART_LIDAR_XYZR=1`
  启动 LIDAR XYZR 调试打印。只用于临时查坐标。

- `RC_AUTOSTART_LIDAR_XYZR=0`
  不启动 LIDAR XYZR 调试打印。默认建议关闭。

- `RC_AUTOSTART_CAN_DURING_MAPPING=1`
  建图模式也启动 CAN。通常不需要。

- `RC_AUTOSTART_CAN_DURING_MAPPING=0`
  建图模式跳过 CAN。默认更安全。

- `RC_AUTOSTART_SKIP_PRESTOP=1`
  启动前不自动清理旧雷达进程。危险，容易造成重复进程，除非明确知道自己在做什么。

- `RC_AUTOSTART_SKIP_PRESTOP=0`
  启动前先执行残余进程清理。默认推荐。

- `RC_AUTOSTART_DRY_RUN=1`
  只打印一键启动会启动哪些节点，不真正启动。适合检查启动计划。

- `RC_AUTOSTART_DRY_RUN=0`
  正常启动。

- `RC_AUTOSTART_SETUP_LIVOX_NET=1`
  Livox 启动前先尝试配置雷达网口。需要免密 sudo/系统配置配合。

- `RC_AUTOSTART_SETUP_LIVOX_NET=0`
  不自动配置雷达网口。网口已经固定配置好时可用。

- `RC_MAP_QUALITY_OVERLAP=1`
  打开地图/当前点云重合率诊断，能帮助判断地图质量，但会增加 CPU 开销，只建议临时诊断。

- `RC_MAP_QUALITY_OVERLAP=0`
  关闭重合率诊断。默认保持 `0`。

- `RC_HEALTH_MONITOR_EVENT_CONTEXT_CSV=1`
  健康日志在尖峰事件附近保存更细的上下文 CSV，便于深挖跳变原因。

- `RC_HEALTH_MONITOR_EVENT_CONTEXT_CSV=0`
  不写详细上下文，只写常规日志和事件摘要。默认更轻量。

配套选择项，不是 0/1，但经常和上面的开关一起用：

- `RC_LIDAR_MODE=mapping`
  建图模式。保存地图前确认 FAST-LIO 正常发布 `/map_save` 服务。

- `RC_LIDAR_MODE=relocalization`
  重定位模式。正式调车和开机自启默认用这个。

- `RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1`
  认为启动点接近地图/机器人业务原点。当前小场地图方案常用这个。

- `RC_POSE_FILTER_SELECTED=normal|stable|beta`
  选择 CAN/生产链路使用哪个滤波结果。选择 `stable` 时要同时开 `RC_POSE_FILTER_STABLE=1`；选择 `beta` 时要同时开 `RC_POSE_FILTER_BETA=1`。

- `RC_CAN_POSE_SOURCE=pose_topic|tf`
  CAN 位姿来源选择。当前生产更倾向 `pose_topic`，也就是发送 `/robot/pose_filtered` 或所选滤波话题；`tf` 主要用于回退排查。

- `RC_ROBOT_POSE_SOURCE=odom|tf`
  `/robot/pose` 的来源选择。当前重定位生产默认是 `odom`：订阅 `/map_lidar/odom`，在同一帧地图定位结果上加雷达到机器人中心外参，计算 `robot_origin -> robot_base` 并发布 `/robot/pose`。`tf` 是回退/对比模式：直接查询 `robot_origin -> robot_base_highrate`，可能混合低频地图修正和高频 TF，当前不建议作为默认调车链路。


# 当前重定位位姿链路说明

当前默认：

```text
/map_lidar/odom
  -> pose_from_odom 加 body -> robot_base 外参
  -> /robot/pose
  -> /robot/pose_filtered 或 /robot/pose_filtered_stable
  -> CAN
```

保留当前 TF 树用于 RQT、tf2_echo、外参校准和调试：

```text
map
├── camera_init
│   ├── body -> robot_base
│   └── body_highrate -> robot_base_highrate
└── robot_origin
```

但生产 `/robot/pose` 默认不再通过 TF 查询 `robot_origin -> robot_base_highrate`。如果想复现或对比 TF 拼接路径：

```bash
RC_ROBOT_POSE_SOURCE=tf ./autostart/start_all_terminals.sh
```
