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
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=0 \
./autostart/start_all_terminals.sh


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


# 大场地建议先裁比赛局部地图，不要直接拿整张大场馆 PCD 当全局重定位用
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
