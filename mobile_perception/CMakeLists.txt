cmake_minimum_required(VERSION 3.5)
project(mobile_perception)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag_ros REQUIRED)  # Ensure this is the correct package name
find_package(nav2_graceful_controller REQUIRED)
find_package(nav2_util REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(apriltag REQUIRED)


find_package(sensor_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)


add_executable(apriltag_detection_node
  src/apriltag_detection_node.cpp
)

ament_target_dependencies(apriltag_detection_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
  nlohmann_json
  std_srvs
)
target_link_libraries(apriltag_detection_node
  apriltag::apriltag
  ${OpenCV_LIBS}
)


install(TARGETS apriltag_detection_node
  DESTINATION lib/${PROJECT_NAME}
)


include_directories(include ${OpenCV_INCLUDE_DIRS})

# mobile_control_node
add_executable(mobile_control_node src/mobile_control_node.cpp)
ament_target_dependencies(mobile_control_node rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs std_srvs)
install(TARGETS
  mobile_control_node
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
ament_export_dependencies(rclcpp rclpy cv_bridge apriltag_ros)

ament_package()
