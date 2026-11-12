# Mission 3：主机部署与视觉角点坐标

目标：由雷达侧操作者在目标主机部署，持续向视觉发布约定角点的X/Y/yaw；视觉组接收坐标并使用UI。
当前程序已完成开发电脑上的软件验证，目标主机环境待报告；现有精度按用户选择先工程试用。

## 两方各自做什么

- 雷达侧（本项目用户）：在主机拉取代码、准备环境、启动驱动/FAST-LIO/角点接口、摆正实体角点和初始方向建立起点。
- 视觉侧：按接口订阅坐标，处理valid、采样时间、reference_id；需要车心时自行换算。可使用已有UI查看/操作允许的起点流程。
- UI能显示数据，不等于它单独完成雷达定位。主机进程持续运行才能持续给视觉坐标。

## 你在目标主机先执行

```bash
git clone --depth 1 --branch mission3 https://github.com/Sa63R/rc_lidar_r2_v2.git rc_mission3
cd rc_mission3/mission3
bash bin/task3 check --out host_environment.json
```

把环境报告给维护者，决定采用已有雷达栈接入或构建包内上游；环境未知时不直接搬开发机install目录。
主机已经clone时使用git pull更新；push是把本地改动上传到GitHub，不是下载到主机。

## 环境配置后使用

```bash
bash bin/task3 build-app
```

主机已有健康驱动和FAST-LIO则保留；否则分别开终端运行bash bin/task3 lidar、bash bin/task3 fastlio。
另开终端运行bash bin/task3 ui，正确车架角点投影对O、初始轴对齐场地轴后建立起点。
无桌面可以运行bash bin/task3 start；视觉接收示例bash bin/task3 receive。

## 说明入口

- README_先读.md：安装包入口和环境边界。
- docs/00_使用说明.html：浏览器离线阅读，包含CAD角点参考图和全部说明。
- docs/01_主机部署与启动.md：依赖、已有上游/包内上游两条路径、启动/停止。
- docs/02_角点计算与坐标约定.md：IMU/body到角点刚体关系、固定初始轴、yaw、平滑。
- docs/03_视觉接收接口.md：/vision/state的JSON、米/弧度、失效和跨机要求。
- docs/04_版本与测试结论.md：真实截图/位移参照与软件验证边界，Z延期、精度标志仍false。

ROS/SDK2和系统依赖按目标环境提供，当前本机基线Ubuntu24.04/ROS2 Jazzy。
任务一真实码盘、任务二场地ASPA和视觉业务先接入均不是启动这个雷达侧接口的前置条件。
不再要求补返回残余试验才能开始工程试用；未测不记通过。

本目录有COLCON_IGNORE以免父雷达工作空间自动扫描到内嵌vendor重复包；bin/task3使用显式源码路径独立构建。
当前仓库分支基于已有Ubuntu24.04/Jazzy分支，原主分支不改。
