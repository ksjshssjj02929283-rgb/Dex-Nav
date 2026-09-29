cmake_minimum_required(VERSION 3.8)
project(auto_charging_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖查找
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(chassis_interfaces REQUIRED)

# 包含目录
include_directories(include)

# 编译库：所有行为树节点实现
add_library(charging_nodes SHARED
  src/auto_charging_bt.cpp
  src/docking_controller.cpp
  src/reverse_docking_controller.cpp
  src/reverse_docking_bt_controller.cpp
)

ament_target_dependencies(charging_nodes
  rclcpp
  rclcpp_action
  diagnostic_msgs
  geometry_msgs
  nav2_msgs
  nav_interfaces
  std_msgs
  behaviortree_cpp_v3
  nlohmann_json
  std_srvs
  ament_index_cpp
  tf2_geometry_msgs
  tf2_ros
  tf2
)

# # 编译可执行文件
# add_executable(auto_charging_bt_node src/main.cpp)
# ament_target_dependencies(auto_charging_bt_node
#   ament_index_cpp
#   rclcpp
#   behaviortree_cpp_v3
# )

# target_link_libraries(auto_charging_bt_node
#   charging_nodes
# )

# 新增：服务形式的可执行文件
add_executable(auto_charging_service_node src/service_node.cpp)
ament_target_dependencies(auto_charging_service_node
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  ament_index_cpp
  nav_interfaces
  std_msgs
  std_srvs
)
target_link_libraries(auto_charging_service_node
  charging_nodes
)

add_executable(auto_charging_action_server_node src/auto_charging_action_server_node.cpp)
ament_target_dependencies(auto_charging_action_server_node
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  ament_index_cpp
  nav_interfaces
  std_msgs
  chassis_interfaces
)
target_link_libraries(auto_charging_action_server_node
  charging_nodes
)

# ==================== 测试节点（非单元测试，用于独立跑控制器） ====================
add_executable(test_odom_docking_controller test/test_odom_docking_controller.cpp)
ament_target_dependencies(test_odom_docking_controller
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(test_odom_docking_controller
  charging_nodes
)

add_executable(test_reverse_docking_controller test/test_reverse_docking_controller.cpp)
ament_target_dependencies(test_reverse_docking_controller
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(test_reverse_docking_controller
  charging_nodes
)

# 安装
install(TARGETS
  charging_nodes
  # auto_charging_bt_node
  auto_charging_service_node
  auto_charging_action_server_node
  test_odom_docking_controller
  test_reverse_docking_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
