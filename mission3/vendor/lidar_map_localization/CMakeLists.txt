cmake_minimum_required(VERSION 3.8)
project(lidar_map_localization)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

add_executable(map_localizer src/map_localizer.cpp)
target_include_directories(map_localizer PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(map_localizer ${PCL_LIBRARIES})
ament_target_dependencies(map_localizer
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
)

add_executable(pcd_quality_report src/pcd_quality_report.cpp)
target_include_directories(pcd_quality_report PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_quality_report ${PCL_LIBRARIES})

add_executable(pcd_prepare_local_map src/pcd_prepare_local_map.cpp)
target_include_directories(pcd_prepare_local_map PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_prepare_local_map ${PCL_LIBRARIES})

install(TARGETS map_localizer pcd_quality_report pcd_prepare_local_map
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
