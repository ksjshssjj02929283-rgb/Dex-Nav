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
find_package(action_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# nav2_msgs 只用于「恢复行为」触发源（默认关闭）。环境里没有时跳过该触发源即可，
# 不让整个包编不出来。
find_package(nav2_msgs QUIET)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TriggerRecord.srv"
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

set(dependencies
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  std_srvs
  action_msgs
  diagnostic_msgs
  nav_msgs
  std_msgs
)

if(nav2_msgs_FOUND)
  list(APPEND dependencies nav2_msgs)
  add_compile_definitions(MOBILE_BAG_RECORDER_HAS_BT_LOG)
  message(STATUS "mobile_bag_recorder: nav2_msgs found, recovery-behavior trigger enabled")
else()
  message(WARNING
    "mobile_bag_recorder: nav2_msgs not found, the recovery-behavior trigger "
    "(trigger.on_recovery_behavior) will not be available")
endif()

add_library(${PROJECT_NAME}_lib
  src/recorder_config.cpp
  src/topic_subscriber_set.cpp
  src/continuous_recorder.cpp
  src/retention_policy.cpp
  src/time_ring_buffer.cpp
  src/event_recorder.cpp
  src/trigger_monitor.cpp
  src/trigger_sources/builtin.cpp
  src/trigger_sources/nav_status_source.cpp
  src/trigger_sources/diagnostics_source.cpp
  src/trigger_sources/goal_error_source.cpp
  src/trigger_sources/nav_stall_source.cpp
  src/trigger_sources/recovery_behavior_source.cpp
  src/mobile_bag_recorder_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

# 同包内既生成接口又有 C++ 库，需显式链接自身的 typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

add_executable(mobile_bag_recorder_node
  apps/mobile_bag_recorder_node.cpp
)

target_link_libraries(mobile_bag_recorder_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(mobile_bag_recorder_node ${dependencies})

# 共享库必须装在 lib/ 而不是 lib/<pkg>/：后者不在 LD_LIBRARY_PATH 中，
# 运行时会报 cannot open shared object file
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mobile_bag_recorder_node
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

install(PROGRAMS
  scripts/inject_trigger.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_retention_policy test/test_retention_policy.cpp)
  target_link_libraries(test_retention_policy ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_retention_policy ${dependencies})

  ament_add_gtest(test_time_ring_buffer test/test_time_ring_buffer.cpp)
  target_link_libraries(test_time_ring_buffer ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_time_ring_buffer ${dependencies})

  ament_add_gtest(test_throttle_rules test/test_throttle_rules.cpp)
  target_link_libraries(test_throttle_rules ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_throttle_rules ${dependencies})
endif()

ament_package()
