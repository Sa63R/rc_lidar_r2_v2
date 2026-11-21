# 任务三 r4：固定地图原点与自动定位

入口：mission3/。Ubuntu24.04 / ROS2 Jazzy。

本版默认run和“任务三角点坐标”菜单会自动在旧地图中定位，输出相对已绑定正式O的角点X/Y/yaw；启动点不再清零，无需鼠标指定当前位置。

源码、地图定位组件、PCL自动搜索、同一张压缩地图及正式O配置一并提供。目标机先更新源码，准备依赖，再依次prepare-field、build-map、build-app、install-launcher；之后运行run或同名菜单。

请在项目文件夹执行这些bin/task3子命令。首次已安装r3的主机保留已有驱动、FAST-LIO与SDK2，不必重新完整build-stack。

本机已接通自动搜索/地图定位/正式O绑定和静止接口；异位冷启动及另一台主机r4运行仍需现场确认。physical_accuracy_verified=false，Z延期。

视觉订阅/vision/state，X/Y米、yaw弧度，UI为度；valid=false或坐标过期就停用旧值。

源码包可从本地“雷达坐标交付1”文件夹取得。GitHub只保留一份源码和压缩地图，不再重复存放旧r3源码归档；历史提交仍可追溯。
