cmake_minimum_required(VERSION 2.8.3)
project(lexicographic_planning)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++11 -O3 -pthread")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(lex_planner SHARED
  src/lex_planner.cpp
)

ament_target_dependencies(lex_planner
  rclcpp
  nav2_core
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core lex_planner_plugin.xml)

install(TARGETS lex_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
