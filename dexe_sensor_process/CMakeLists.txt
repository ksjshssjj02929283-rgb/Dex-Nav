cmake_minimum_required(VERSION 3.8)
project(dexe_sensor_process)

# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

## RGBD to PointCloud2 converter
add_executable(rgbd_to_pointcloud 
  src/rgbd_to_pointcloud.cpp
)

ament_target_dependencies(rgbd_to_pointcloud
  rclcpp
  sensor_msgs
  std_msgs
  message_filters
  PCL
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
)

target_include_directories(rgbd_to_pointcloud PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_link_directories(rgbd_to_pointcloud PUBLIC
  ${PCL_LIBRARY_DIRS}
)

target_link_libraries(rgbd_to_pointcloud
  ${PCL_LIBRARIES}
)

install(TARGETS
  rgbd_to_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
