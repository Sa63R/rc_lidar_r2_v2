cmake_minimum_required(VERSION 3.8)
project(rc_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/fake_diff_drive.py
  scripts/pcd_to_occupancy.py
  tools/create_manual_field_map.py
  tools/draw_manual_obstacles.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
