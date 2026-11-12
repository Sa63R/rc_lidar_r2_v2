# 任务三 · 角点坐标交付包

版本：2026-11-12 r1（工程试用交付）。接收方：视觉组。

本包让主机运行雷达定位，计算约定角点的X、Y、yaw，并由视觉订阅。已有桌面UI用于建立起点、
显示坐标、查看时效和停止本窗口启动的接口。主机环境尚未提供，不能把本包已生成说成目标主机已部署。

## 先看哪份文件

1. 浏览器打开docs/00_使用说明.html，先理解启动顺序和接口。
2. 主机操作人员看docs/01_主机部署与启动.md；视觉开发人员看docs/03_视觉接收接口.md。
3. 解释“坐标怎么算”看docs/02_角点计算与坐标约定.md。
4. 测试范围与当前限制见docs/04_版本与测试结论.md。

## 主机环境未知时先做什么

解压到一个长期保留、用户可写的目录，进入解压目录，运行：

```bash
bash bin/task3 check --out host_environment.json
```

把host_environment.json交给雷达开发人员判断安装路径。检查仅读取系统环境，不连接雷达、不安装软件。
本机验证基线是Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12；其他ROS版本、Windows或不同架构没有实机部署保证。

## 已有雷达驱动和FAST-LIO时

复制config/deploy.env.example为config/deploy.env，将TASK3_BASE_SETUP填写为目标主机已有雷达工作空间
的install/setup.bash绝对路径，再运行：

```bash
bash bin/task3 build-app
bash bin/task3 ui
```

需要现有驱动和FAST-LIO已输出正确的/Odometry（camera_init→body）。若已有其他坐标定义，先适配，不能只改话题名。

## 如何把坐标给视觉

雷达主机运行发布程序，视觉程序订阅/vision/state（ROS 2 std_msgs/msg/String中的JSON）。
读取data.x、data.y（米）、data.yaw（弧度），遵守valid、stamp_ns和reference_id。

接收示例：

```bash
bash bin/task3 receive
```

同机只需订阅，不用复制数字。跨机还需要ROS网络互通、相同ROS域和同步时钟；真实跨机情况尚未验证。
程序默认支持ROS 2接入；视觉若只用非ROS环境，需要先说明接收方式，再增加对应适配层。

## 本包边界

- 输出约定的**车架刚性角点**，不是轮胎接地点，也不是车心。视觉需要车心时自行用真实角点→车心向量换算。
- 每次新的FAST-LIO会话，把正确角点的地面投影对准正式O，车体图示轴对齐场地轴，再建立起点。
- 多轮连续运行保留原点；断电/定位重初始化或改用另一实体点后须重新按正确姿态建立起点。
- 有效输出为X/Y/yaw；Z未作有效高度交付。物理精度标志仍false；用户选择现有精度先交付试用，不补返回残余试验。
- 本包不携带历史起点、地图、原始录包或账号资料。包含当前源码/配置、FAST-LIO与驱动源码、说明与示例。
- ROS、编译依赖、Livox-SDK2需要目标主机提供或按说明安装；这不是含操作系统与全部依赖的离线镜像。

运行bash bin/task3 help查看入口。文件清单和SHA256见MANIFEST.json，可用python3 tools/verify_release.py校验。
