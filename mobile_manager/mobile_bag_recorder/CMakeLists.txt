cmake_minimum_required(VERSION 3.8)
project(mobile_bag_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  std_srvs
)

add_library(${PROJECT_NAME}_lib
  src/recorder_config.cpp
  src/topic_subscriber_set.cpp
  src/continuous_recorder.cpp
  src/retention_policy.cpp
  src/mobile_bag_recorder_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

add_executable(mobile_bag_recorder_node
  apps/mobile_bag_recorder_node.cpp
)

target_link_libraries(mobile_bag_recorder_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(mobile_bag_recorder_node ${dependencies})

install(TARGETS
  ${PROJECT_NAME}_lib
  mobile_bag_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_retention_policy test/test_retention_policy.cpp)
  target_link_libraries(test_retention_policy ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_retention_policy ${dependencies})
endif()

ament_package()
