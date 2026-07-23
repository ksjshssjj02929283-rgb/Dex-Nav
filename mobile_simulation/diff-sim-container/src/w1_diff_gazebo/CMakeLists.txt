cmake_minimum_required(VERSION 3.8)
project(w1_diff_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch scripts meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
