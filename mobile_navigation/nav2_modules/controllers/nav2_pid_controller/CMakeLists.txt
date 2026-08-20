cmake_minimum_required(VERSION 3.5)
project(nav2_pid_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_controller_common REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(dependencies
  angles
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  nav_interfaces
  nav2_costmap_2d
  nav2_util
  nav2_core
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

set(library_name nav2_pid_controller)

add_library(${library_name} SHARED
  src/end_localization_lock.cpp
  src/pid_controller.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  nav2_controller_common::nav2_controller_common
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pid_speed_loop test/test_pid_speed_loop.cpp)
  target_link_libraries(test_pid_speed_loop ${library_name})
  ament_target_dependencies(test_pid_speed_loop ${dependencies})

  ament_add_gtest(test_end_localization_lock test/test_end_localization_lock.cpp)
  target_link_libraries(test_end_localization_lock ${library_name})
  ament_target_dependencies(test_end_localization_lock ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_dependencies(nav2_controller_common)

pluginlib_export_plugin_description_file(nav2_core nav2_pid_controller.xml)

ament_package()
