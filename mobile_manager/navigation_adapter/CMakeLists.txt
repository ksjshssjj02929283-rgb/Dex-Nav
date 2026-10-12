cmake_minimum_required(VERSION 3.8)
project(navigation_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  src/navigation_adapter_node.cpp
  src/asset_navigation_adapter/asset_navigation_adapter.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  std_msgs
  nav_msgs
  nav2_msgs
  nav_interfaces
  geometry_msgs
)

add_executable(navigation_adapter_node
  apps/navigation_adapter_node.cpp
)

target_link_libraries(navigation_adapter_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(navigation_adapter_node
  rclcpp
  rclcpp_action
  std_msgs
  nav_msgs
  nav2_msgs
  nav_interfaces
  geometry_msgs
)

install(TARGETS
  ${PROJECT_NAME}_lib
  navigation_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
