# 任务三：视觉组独立使用

当前为2026-11-14 r3。首次部署完成后，视觉组可自行启动主机、建立原点和使用坐标，不需要原开发者每次在场。

## 主机已经克隆并编译过旧版

在原主机逐条执行：

```bash
cd ~/rc_mission3
```

```bash
git pull --ff-only origin mission3
```

```bash
cd mission3
```

```bash
bash bin/task3 install-launcher
```

核心算法和机械参数未变，沿用已编译结果；本次新增一键启动、UI选项与说明，无需重编译驱动/FAST-LIO。
若git提示本地改动冲突，保留改动并处理，不使用reset --hard或强推。
已经用交付文件夹的更新主机.sh更新成功时，不用重复更新运行程序。

## 日常使用

主机应用菜单搜索“任务三角点坐标”，或在本目录执行：

```bash
ROS_DOMAIN_ID=0 bash bin/task3 run
```

保持车静止，等待“定位已就绪”。正确车架角点投影对正式O，初始图示轴对齐场地轴，再点“建立起点并启动”。
有效坐标和起点编号出现后再移车。结束时先让视觉停用坐标，再关闭窗口并等待自建进程退出。
断电/定位重启或输入中断后，需要按正式O与初始方向重新建立起点，不会自动恢复场地坐标。

## 说明与接口

浏览器打开docs/00_使用说明.html。接收示例tools/vision_subscriber.py；主通道/vision/state，读取data.x/y（米）和data.yaw（弧度）。
UI用度。超时或无效时停用旧值，起点改变后清除旧轨迹。跨机接收需ROS网络互通和同步时钟。

全新主机先看docs/01_主机部署与启动.md，准备ROS/SDK2依赖并build-stack，不跨电脑复制install目录。
仓库根目录的task3_corner_vision_20261114_r3.tar.gz与交付文件夹的源码包一致，可用于文件更新。
主机下载更新用clone/pull；push是上传到GitHub。

55项相关软件测试、新目录应用构建和隔离ROS合成UI流程通过；目标主机新入口实际运行仍待恢复供电后确认。
输出指定角点X/Y/yaw，Z延期，physical_accuracy_verified如实false。软件验证不代表新实车精度。
