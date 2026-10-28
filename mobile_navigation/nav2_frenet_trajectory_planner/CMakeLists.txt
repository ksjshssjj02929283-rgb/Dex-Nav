cmake_minimum_required(VERSION 3.8)
project(nav2_frenet_ilqr_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

set(library_name nav2_frenet_local_planner)

add_library(${library_name} SHARED
  src/frenet_local_planner.cpp
  src/frenet_trajectory_planner.cpp
  src/frenet_policies.cpp
  src/frenet_costs.cpp
)

ament_target_dependencies(${library_name}
  angles
  eigen3_cmake_module
  geometry_msgs
  nav2_costmap_2d
  nav2_core
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(${library_name} Eigen3::Eigen)

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(
  angles
  eigen3_cmake_module
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)
ament_export_targets(${library_name})

pluginlib_export_plugin_description_file(nav2_core nav2_frenet_ilqr_controller.xml)
pluginlib_export_plugin_description_file(
  nav2_frenet_ilqr_controller frenet_policies.xml)
pluginlib_export_plugin_description_file(
  nav2_frenet_ilqr_controller frenet_costs.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frenet_trajectory_planner test/test_frenet_trajectory_planner.cpp)
  target_link_libraries(test_frenet_trajectory_planner ${library_name})
  ament_target_dependencies(test_frenet_trajectory_planner
    nav2_core
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
  )
  ament_add_gtest(test_frenet_plugins test/test_frenet_plugins.cpp)
  target_link_libraries(test_frenet_plugins ${library_name})
  ament_target_dependencies(test_frenet_plugins pluginlib)
endif()

ament_package()
