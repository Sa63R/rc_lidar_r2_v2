# 雷达pcd的rviz可视化

cd /home/jwt/jwt/rc/rc_lidar_r2_v2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run pcl_ros pcd_to_pointcloud \
  --ros-args \
  -p file_name:=/home/jwt/jwt/rc/rc_lidar_r2_v2/maps/mid360_map_20260625_005422.pcd \
  -p tf_frame:=map \
  -p publishing_period_ms:=1000 \
  -r cloud_pcd:=/pcd_map
# source命令，泛用

cd /home/jwt/jwt/rc/rc_lidar_r2_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2

# r2的图
/home/jwt/jwt/rc/rc_lidar_r2_v2/maps/mid360_map_20260625_005422.pcd
# r1的图
/home/jwt/jwt/rc/rc_lidar_r2_v2/maps/mid360_map_20260625_064655.pcd

# 启动图片
ros2 launch rc_nav2_bringup fake_nav.launch.py \
  map:=/home/jwt/jwt/rc/rc_lidar_r2_v2/src/rc_nav2_bringup/maps/mid360_064655_nav2_low.yaml