cmake_minimum_required(VERSION 3.10)
project(map_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_IMAGE REQUIRED SDL2_image)
find_package(glog REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
  ${SDL2_IMAGE_INCLUDE_DIRS}
)

add_library(map_manager_core
  src/image_loader.cpp
  src/map_manager.cpp
  src/map_manager_config.cpp
)

ament_target_dependencies(map_manager_core
  rclcpp
  nav_msgs
  tf2
  ament_index_cpp
)

target_link_libraries(map_manager_core
  Boost::filesystem
  glog::glog
  ${SDL2_LIBRARIES}
  ${SDL2_IMAGE_LIBRARIES}
  yaml-cpp
)

add_executable(map_manager_node
  apps/map_manager_node.cpp
  apps/map_manager_ros.cpp
)

ament_target_dependencies(map_manager_node
  rclcpp
  nav_msgs
  std_srvs
  std_msgs
  nav_interfaces
  tf2_ros
  tf2
  tf2_geometry_msgs
)

target_link_libraries(map_manager_node
  map_manager_core
)

install(TARGETS
  map_manager_core
  map_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

