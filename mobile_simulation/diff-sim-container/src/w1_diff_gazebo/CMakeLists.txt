cmake_minimum_required(VERSION 3.8)
project(w1_diff_gazebo)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(sim_cmd_vel_bridge src/sim_cmd_vel_bridge.cpp)
ament_target_dependencies(sim_cmd_vel_bridge geometry_msgs rclcpp)

install(DIRECTORY urdf launch scripts meshes config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS sim_cmd_vel_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
