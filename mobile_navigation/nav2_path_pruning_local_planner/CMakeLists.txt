cmake_minimum_required(VERSION 3.5)
project(nav2_path_pruning_local_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

set(dependencies
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros)

add_library(${PROJECT_NAME} SHARED
  src/path_pruning_local_planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_path_pruning_local_planner
    test/test_path_pruning_local_planner.cpp)
  target_link_libraries(test_path_pruning_local_planner ${PROJECT_NAME})
  ament_target_dependencies(test_path_pruning_local_planner ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core nav2_path_pruning_local_planner.xml)

ament_package()
