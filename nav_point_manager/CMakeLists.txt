cmake_minimum_required(VERSION 3.8)
project(nav_point_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(
  include
)

# 创建库
add_library(${PROJECT_NAME}_lib
  apps/file_manager.cpp
  apps/point_database.cpp
  apps/navigation_executor.cpp
  apps/nav_point_manager_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  std_srvs
  std_msgs
  geometry_msgs
  visualization_msgs
  nav2_msgs
  nav_msgs
  tf2
  nav_interfaces
  rcl_interfaces
)

target_link_libraries(${PROJECT_NAME}_lib
  nlohmann_json::nlohmann_json
)

# Nav point manager node
add_executable(nav_point_manager_node
  apps/main.cpp
)

target_link_libraries(nav_point_manager_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(nav_point_manager_node
  rclcpp
)

# Install targets
install(TARGETS
  nav_point_manager_node
  ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  params
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
