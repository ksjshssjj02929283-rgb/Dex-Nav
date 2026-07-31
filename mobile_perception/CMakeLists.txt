cmake_minimum_required(VERSION 3.5)
project(mobile_perception)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag_ros REQUIRED)  # Ensure this is the correct package name
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(apriltag REQUIRED)
find_package(sensor_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ruckig REQUIRED)

add_executable(charging_pose_service_node src/charging_pose_service.cpp)
ament_target_dependencies(charging_pose_service_node
  rclcpp std_srvs tf2_ros geometry_msgs tf2_geometry_msgs nlohmann_json
)
install( TARGETS
    charging_pose_service_node
    DESTINATION lib/${PROJECT_NAME}) 


add_executable(apriltag_detection_node
  src/apriltag_detection_node.cpp
)

ament_target_dependencies(apriltag_detection_node
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
  nlohmann_json
)
target_link_libraries(apriltag_detection_node
  apriltag::apriltag
  ${OpenCV_LIBS}
)

add_executable(apriltag_alignment_tf_node
  src/apriltag_alignment_tf_node.cpp
)

ament_target_dependencies(apriltag_alignment_tf_node
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
  nlohmann_json
)
target_link_libraries(apriltag_alignment_tf_node
  apriltag::apriltag
  ${OpenCV_LIBS}
)


install(TARGETS
  apriltag_detection_node
  apriltag_alignment_tf_node
  DESTINATION lib/${PROJECT_NAME}
)


include_directories(include ${OpenCV_INCLUDE_DIRS})

# mobile_control_node
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/mobile_control_node.cpp")
  add_executable(mobile_control_node src/mobile_control_node.cpp)
  ament_target_dependencies(
    mobile_control_node
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    nav_msgs
    std_srvs
  )
  install(TARGETS
    mobile_control_node
    DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "Skipping legacy mobile_control_node: src/mobile_control_node.cpp is absent")
endif()

# Costmap-free precise-control algorithms
add_library(mobile_control_controllers SHARED
  src/mobile_control/straight_controller.cpp
  src/mobile_control/rotation_controller.cpp
  src/mobile_control/goal_checker.cpp
  src/mobile_control/s_curve_speed_profile_planner.cpp
)
target_include_directories(mobile_control_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mobile_control_controllers ruckig::ruckig)

# Short Path + S profile -> RobotTrajectory projection
add_library(mobile_control_trajectory SHARED
  src/mobile_control/short_trajectory_builder.cpp
)
target_include_directories(mobile_control_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  mobile_control_trajectory
  geometry_msgs
  nav_msgs
  nav_interfaces
  tf2
  tf2_geometry_msgs
)

# pluginlib-only factory. The implementation remains in the algorithm library,
# so the action node never links a Nav2 speed-planner/costmap plugin.
add_library(mobile_control_speed_profile_plugins SHARED
  src/mobile_control/s_curve_speed_profile_plugin.cpp
)
target_include_directories(mobile_control_speed_profile_plugins PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(mobile_control_speed_profile_plugins pluginlib)
target_link_libraries(
  mobile_control_speed_profile_plugins
  mobile_control_controllers
)
pluginlib_export_plugin_description_file(
  mobile_perception
  mobile_control_speed_profile_plugins.xml
)

# mobile_control_action_node
add_executable(mobile_control_action_node src/mobile_control_action_node.cpp)
ament_target_dependencies(
  mobile_control_action_node
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  nav_interfaces
  pluginlib
)
target_link_libraries(
  mobile_control_action_node
  mobile_control_controllers
  mobile_control_trajectory
)
install(TARGETS
  mobile_control_action_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  mobile_control_controllers
  mobile_control_trajectory
  mobile_control_speed_profile_plugins
  EXPORT export_mobile_control_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# obstacle_detector_node
add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
ament_target_dependencies(obstacle_detector_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)
install(TARGETS
  obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/detect_apriltags.py
  scripts/detect_apriltag_nostop.py
  scripts/detect_apriltags_service.py
  scripts/control_robot.py
  scripts/nav_then_precise.py
  scripts/nav_then_precise_bt.py
  scripts/goal_controller.py
  scripts/stop_robot.py
  scripts/spin_robot.py
  scripts/undocking.py
  scripts/control_robot_simulation.py
  scripts/goal_controller_simulation.py
  scripts/nav_goal_publisher.py
  scripts/nav_goal_publisher_loop.py
  scripts/pub_current_position.py
  scripts/pub_goal_loop_office.py
  scripts/create_charging_pose.py
  scripts/nav_then_precise_bt.py
  scripts/nav_goal_server.py
  DESTINATION lib/${PROJECT_NAME})

# Install directories
install(DIRECTORY
  launch
  config
  images
  DESTINATION share/${PROJECT_NAME})

# Setup include directories
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclpy
  cv_bridge
  apriltag_ros
  nav_interfaces
  pluginlib
  ruckig
)
ament_export_targets(export_mobile_control_controllers HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test utility: dump the production S planner + runtime tracking sampler +
  # StraightController/RotationController target-speed result as CSV. The
  # companion Python script only renders this output.
  add_executable(
    mobile_control_speed_profile_dump
    test/mobile_control_speed_profile_dump.cpp
  )
  target_link_libraries(
    mobile_control_speed_profile_dump
    mobile_control_controllers
  )

  ament_add_gtest(
    test_mobile_control_controllers
    test/test_mobile_control_controllers.cpp
  )
  target_link_libraries(
    test_mobile_control_controllers
    mobile_control_controllers
    mobile_control_trajectory
  )
  ament_target_dependencies(
    test_mobile_control_controllers
    geometry_msgs
    nav_msgs
    nav_interfaces
    tf2
    tf2_geometry_msgs
  )

  install(
    TARGETS mobile_control_speed_profile_dump
    DESTINATION lib/${PROJECT_NAME}
  )
  install(
    PROGRAMS test/plot_mobile_control_speed_profiles.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
