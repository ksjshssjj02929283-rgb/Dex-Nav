cmake_minimum_required(VERSION 3.8)
project(w1_diff_gazebo)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(sim_cmd_vel_bridge src/sim_cmd_vel_bridge.cpp)
ament_target_dependencies(sim_cmd_vel_bridge geometry_msgs rclcpp)

add_executable(w1_diff_kinematic_emulator src/w1_diff_kinematic_emulator.cpp)
ament_target_dependencies(w1_diff_kinematic_emulator
  geometry_msgs
  nav_msgs
  sensor_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(DIRECTORY urdf launch scripts meshes config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS sim_cmd_vel_bridge w1_diff_kinematic_emulator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
