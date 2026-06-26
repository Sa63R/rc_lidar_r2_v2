# codex 不对这个文件做记录，但是必要错误可修改

# TEST
## 性能测试
chromium-browser --proxy-server="http://127.0.0.1:7897"


# 关残余雷达进程

   cd '/home/rc2/jwt_new_lidar/rc_lidar_r2_v2'
./autostart/stop_all.sh

# 当前可用版本：小场地直接建图，不裁剪，使用这张地图重定位
# 地图：/home/rc2/jwt_new_lidar/rc_lidar_r2_v2/maps/mid360_map_20260625_005422.pcd
# 启动前提：车放在建图原点附近，所以使用 RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1
# 重定位调试/开机启动：核心 headless，CANFD 发 /robot/pose_continuous；有显示器时打开 RViz/PlotJuggler，没显示器时自动跳过 GUI
cd "/home/rc2/jwt_new_lidar/rc_lidar_r2_v2"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=1 \
RC_AUTOSTART_CANFD=1 \
RC_CAN_DEBUG_CSV=1 \
FASTLIO_RVIZ=true \
RC_LIDAR_MAP_PATH="/home/rc2/jwt_new_lidar/rc_lidar_r2_v2/maps/mid360_map_20260625_005422.pcd" \
./autostart/start_all_terminals.sh



<!-- 旋转自校正脚本 -->

cd '/home/rc2/jwt_new_lidar/rc_lidar_r2_v2'
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fastlio_world_init rotation_extrinsic_calibrator \
  --duration 40 \
  --rate 50


建图模块

# 建图
cd "/home/rc2/jwt_new_lidar/rc_lidar_r2_v2"
RC_LIDAR_MODE=mapping ./autostart/start_all_terminals.sh

# 保存地图
cd "/home/rc2/jwt_new_lidar/rc_lidar_r2_v2"
./autostart/save_fast_lio_map.sh

地图默认保存到：
maps/mid360_map.pcd




## 旧滤波/旧 yaw fusion 调试参数
当前默认不启动 `/robot/pose_filtered` 和 `/robot/yaw_fusion/status`，CANFD 直接发 `/robot/pose_continuous`。下面这些只在手动打开旧链路对比时使用。
<!-- 越大越接近raw,但是过不了门控 -->
RC_POSE_FILTER=1
RC_POSE_FILTER_YAW_ALPHA=0.85

<!-- 可以过门控 -->
RC_POSE_FILTER_USE_RAW_YAW=1
<!-- 走低延迟odom+map_lidar融合yaw -->
RC_ROBOT_POSE_YAW_FUSION=1
RC_CAN_USE_YAW_POSE_TOPIC=1
# temperature

cat


# 常用控制变量速查（精简版）

只保留最常用的 10 个以内。现场调车优先低负载：headless、不开 PlotJuggler、指定小场地图，默认不启动旧 filter/yaw fusion 诊断链路。

## 当前开机/调试重定位

```bash
cd "/home/rc2/jwt_new_lidar/rc_lidar_r2_v2"
RC_LIDAR_MODE=relocalization \
RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1 \
RC_AUTOSTART_HEADLESS=1 \
RC_AUTOSTART_PLOTJUGGLER=1 \
RC_AUTOSTART_CANFD=1 \
RC_CAN_DEBUG_CSV=1 \
FASTLIO_RVIZ=true \
RC_LIDAR_MAP_PATH="/home/rc2/jwt_new_lidar/rc_lidar_r2_v2/maps/mid360_map_20260625_005422.pcd" \
./autostart/start_all_terminals.sh
```

开机自启入口使用 user systemd：`./autostart/install_user_systemd_autostart.sh`。旧的 LiDAR 桌面 XDG 自启动项需要禁用，避免和 systemd 重复启动。

FD CAN 重启功能仍在：监听 FD CAN ID `0x30`，payload 为 `A5 5A 01 FE 00 00 00 00`。当前默认由 `CANFD_POSE` 一边发送 `/robot/pose_continuous`，一边监听这条重启帧；`CAN_RESTART` 只是没有 CANFD 时的手动兜底，不默认启动。重启链路带 8s 节点 debounce 和 15s 脚本 cooldown，防止连续触发把半启动中的雷达栈反复杀掉。

## 最常用变量

1. `RC_LIDAR_MODE=mapping|relocalization`
   `mapping` 建图，`relocalization` 重定位。调车/开机自启默认用 `relocalization`。

2. `RC_LIDAR_MAP_PATH=...`
   指定重定位使用哪张 PCD 地图。换地图时最常改这个。

3. `RC_RELOCALIZATION_ASSUME_MAP_ORIGIN=1|0`
   `1` 表示启动点接近地图/机器人业务原点；当前小场地图方案常用 `1`。

4. `RC_AUTOSTART_HEADLESS=1|0`
   `1` 无头低负载后台启动；`0` 更偏人工调试。开机自启/无显示器用 `1`。

5. `RC_AUTOSTART_PLOTJUGGLER=1|0`
   `1` 自动打开 PlotJuggler 看曲线；`0` 不打开。开机当前设为 `1`，但无显示器或 X 不可达时会自动跳过。

6. `RC_POSE_FILTER=1|0`
   当前默认 `0`，不启动旧 `/robot/pose_filtered`。需要临时对比旧滤波时再设为 `1`。

7. `RC_ROBOT_POSE_YAW_FUSION=1|0`
   当前默认 `0`，不发布旧 `/robot/yaw_fusion/status`。需要对比旧 yaw fusion 时再设为 `1`。

8. `RC_CAN_REQUIRE_READY=1|0`
   `1` CAN 等重定位 ready 后再发；正式调车建议 `1`。`0` 只适合排查 CAN 通不通。

9. `RC_HEALTH_MONITOR=1|0`
   `1` 写健康日志，记录温度、负载、地图质量和位姿；开销小，建议默认 `1`。

10. `RC_ROBOT_POSE_SOURCE=odom|tf`
    `/robot/pose` 来源。当前生产默认 `odom`，从 `/map_lidar/odom` 同帧计算机器人中心；`tf` 只用于对比旧 TF 拼接路径。

当前默认生产链路：

```text
/map_lidar/odom
  -> pose_from_odom 加 body -> robot_base 外参
  -> /robot/pose
/Odometry
  -> continuous_robot_pose 用 map localizer 校正 FAST-LIO 高频 odom
  -> /robot/pose_continuous
  -> CANFD
```

当前坐标约定：

```text
map -> robot_origin xyz = [0.237488, -0.272804, -0.483]
body -> robot_base yaw = -90 deg
map -> robot_origin yaw = -90 deg
robot_base / robot_origin 业务坐标: X 右, Y 前, Z 上
```


cd "/home/rc2/jwt_new_lidar/rc_lidar_r2_v2"
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 - <<'PY'
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def yaw_from_q(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

class YawPrinter(Node):
    def __init__(self):
        super().__init__('print_can_yaw')
        self.create_subscription(PoseStamped, '/robot/pose', self.cb, 10)

    def cb(self, msg):
        yaw = yaw_from_q(msg.pose.orientation)
        print(f"can_yaw_rad={yaw:+.4f}  can_yaw_deg={math.degrees(yaw):+.1f}", flush=True)

rclpy.init()
node = YawPrinter()
rclpy.spin(node)
PY






cd "/home/rc2/jwt_new_lidar/rc_lidar_r2_v2"
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 - <<'PY'
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

def deg(x):
    if not math.isfinite(x):
        return float("nan")
    return math.degrees(x)

class YawFusionPrinter(Node):
    def __init__(self):
        super().__init__("yaw_fusion_printer")
        self.create_subscription(
            Float32MultiArray,
            "/robot/yaw_fusion/status",
            self.cb,
            10,
        )

    def cb(self, msg):
        d = list(msg.data)
        if len(d) < 10:
            print("status too short:", d, flush=True)
            return

        enabled = d[0]
        ready = d[1]
        source = d[2]
        fast_age = d[3]
        map_age = d[4]
        map_ref_yaw = d[5]
        fast_yaw = d[6]
        fused_yaw = d[7]
        delta_yaw = d[8]
        alpha = d[9]

        source_name = {
            1.0: "map_update",
            2.0: "fast_update",
        }.get(source, f"{source:.0f}")

        print(
            f"src={source_name:11s} "
            f"ready={ready:.0f} "
            f"fast_age={fast_age*1000:6.1f}ms "
            f"map_age={map_age*1000:6.1f}ms "
            f"fast={deg(fast_yaw):8.2f}deg "
            f"map_ref={deg(map_ref_yaw):8.2f}deg "
            f"fused={deg(fused_yaw):8.2f}deg "
            f"delta={deg(delta_yaw):8.2f}deg "
            f"alpha={alpha:.3f}",
            flush=True,
        )

rclpy.init()
node = YawFusionPrinter()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PY


## 打应map relocalizer的xy,观测回退现象

cd "/home/rc2/jwt_new_lidar/rc_lidar_r2_v2"
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 - <<'PY'
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class MapLocalizerXYPrinter(Node):
    def __init__(self):
        super().__init__("print_map_localizer_xy")
        self.last_t = None
        self.count = 0
        self.create_subscription(Odometry, "/map_lidar/odom", self.cb, 10)
        print("[map_xy] listening: /map_lidar/odom", flush=True)

    def cb(self, msg):
        now = time.monotonic()
        hz = "--"
        if self.last_t is not None:
            dt = now - self.last_t
            if dt > 0:
                hz = f"{1.0 / dt:5.1f}"
        self.last_t = now
        self.count += 1

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        print(
            f"#{self.count:06d} "
            f"x={p.x:+.4f}m "
            f"y={p.y:+.4f}m "
            f"z={p.z:+.4f}m "
            f"yaw={yaw:+.4f}rad "
            f"yaw={math.degrees(yaw):+7.2f}deg "
            f"hz={hz} "
            f"stamp={stamp:.3f}",
            flush=True,
        )

rclpy.init()
node = MapLocalizerXYPrinter()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY

另外，我现在的启动逻辑，为什么雷达每次启动时候odom发布的雷达当前坐标相对于camera_init不是000,而是会偏几厘米，而且每次的偏差还不一样
