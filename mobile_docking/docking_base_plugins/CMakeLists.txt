cmake_minimum_required(VERSION 3.8)
project(docking_base_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(docking_base_core REQUIRED)
find_package(docking_base_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  pluginlib
  geometry_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  nav_interfaces
  nav2_util
  nav2_core
  nav2_costmap_2d
  docking_base_core
  docking_base_msgs
)

include_directories(include)

# AprilTagPerception 感知插件库（DEC-011 Part1；库名须与 plugins.xml 一致：apriltag_perception）
add_library(apriltag_perception SHARED src/perception/apriltag_perception.cpp)
ament_target_dependencies(apriltag_perception ${dependencies})

# DockingInterface 任务插件（DEC-011 Part2；库名须与 plugins.xml 一致）
add_library(docking_charging SHARED src/docking/docking_charging.cpp)
ament_target_dependencies(docking_charging ${dependencies})
add_library(docking_exit SHARED src/docking/docking_exit.cpp)
ament_target_dependencies(docking_exit ${dependencies})

# 直线对接局部规划插件（库名须与 plugins.xml 一致：straight_line_planner）
add_library(straight_line_planner SHARED src/planner/straight_line_planner.cpp)
ament_target_dependencies(straight_line_planner ${dependencies})

pluginlib_export_plugin_description_file(docking_base_core plugins.xml)

install(TARGETS apriltag_perception docking_charging docking_exit straight_line_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# P1 功能测试 exe：直接实例化 AprilTagPerception，发假检测验 getDockPose
add_executable(perception_test test/test_apriltag_perception.cpp)
target_link_libraries(perception_test apriltag_perception)
ament_target_dependencies(perception_test ${dependencies})
install(TARGETS perception_test DESTINATION lib/${PROJECT_NAME})

# P2 功能测试 exe：驱动 DockingCharging 7 阶段状态机
add_executable(docking_interface_test test/test_docking_charging.cpp)
target_link_libraries(docking_interface_test docking_charging)
ament_target_dependencies(docking_interface_test ${dependencies})
install(TARGETS docking_interface_test DESTINATION lib/${PROJECT_NAME})

# 机器人模拟器（cmd_vel 积分→odom/tf）：与 docking_server 一起 launch 跑端到端
# 注：P3 原型 docking_coordinator 已删（合并进 docking_base/docking_server，DEC-012/013）
add_executable(fake_robot test/test_fake_robot.cpp)
ament_target_dependencies(fake_robot ${dependencies})
install(TARGETS fake_robot DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(apriltag_perception docking_charging docking_exit straight_line_planner)
ament_export_dependencies(${dependencies})

ament_package()
