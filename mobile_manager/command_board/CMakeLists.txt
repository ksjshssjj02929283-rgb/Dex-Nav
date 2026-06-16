cmake_minimum_required(VERSION 3.10)
project(command_board)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PkgConfig REQUIRED)
find_package(glog REQUIRED)

include_directories(
  include
  include/fsm
)


add_library(command_board_core
  src/fsm/localization_state_sm.cpp
)

ament_target_dependencies(command_board_core
  nav2_msgs
  action_msgs
  rclcpp_action
  tf2
)

target_link_libraries(command_board_core
  Boost::filesystem
  glog::glog
  yaml-cpp
)

add_executable(command_board_node
  apps/command_board_node.cpp
  apps/command_board_ros.cpp
)

ament_target_dependencies(command_board_node
  rclcpp
  nav2_msgs
  action_msgs
  rclcpp_action
  diagnostic_msgs
  std_srvs
  std_msgs
  nav_interfaces
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

target_link_libraries(command_board_node
  command_board_core
)

install(TARGETS
  command_board_core
  command_board_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
