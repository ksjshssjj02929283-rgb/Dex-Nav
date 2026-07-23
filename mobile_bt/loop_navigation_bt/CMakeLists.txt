cmake_minimum_required(VERSION 3.8)
project(loop_navigation_bt)

# 依赖查找
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(chassis_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 包含目录
include_directories(include)

# 编译库：所有行为树节点实现
add_library(loop_navigation_nodes SHARED
  src/loop_navigation_bt.cpp
  src/loop_charging_bt.cpp
)

ament_target_dependencies(loop_navigation_nodes
  rclcpp
  rclcpp_action
  sensor_msgs
  diagnostic_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  nlohmann_json
  std_srvs
  chassis_interfaces
  tf2_ros
  tf2_geometry_msgs
)

# 编译可执行文件
add_executable(loop_navigation_bt_node src/main.cpp)
ament_target_dependencies(loop_navigation_bt_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  nlohmann_json
  std_srvs
  chassis_interfaces
  ament_index_cpp
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(loop_navigation_bt_node
  loop_navigation_nodes
)

# 无充电循环导航可执行文件
add_executable(loop_nocharging_bt_node src/main_loop_nocharging.cpp)
ament_target_dependencies(loop_nocharging_bt_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  nlohmann_json
  std_srvs
  chassis_interfaces
  ament_index_cpp
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(loop_nocharging_bt_node
  loop_navigation_nodes
)

# 安装
install(TARGETS
  loop_navigation_nodes
  loop_navigation_bt_node
  loop_nocharging_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/set_navigation_points.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
