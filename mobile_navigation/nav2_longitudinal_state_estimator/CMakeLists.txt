cmake_minimum_required(VERSION 3.8)
project(nav2_longitudinal_state_estimator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_util REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LongitudinalState.msg"
  DEPENDENCIES std_msgs
)

set(core_library ${PROJECT_NAME}_core)
add_library(${core_library} SHARED
  src/longitudinal_state_adapter.cpp
)
target_include_directories(${core_library} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${core_library} rclcpp)

set(node_library ${PROJECT_NAME}_node)
add_library(${node_library} SHARED
  src/longitudinal_state_estimator_node.cpp
)
target_include_directories(${node_library} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${node_library}
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  nav2_util
  geometry_msgs
  nav_msgs
  diagnostic_msgs
)
target_link_libraries(${node_library} ${core_library})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${node_library} "${cpp_typesupport_target}")
rclcpp_components_register_nodes(
  ${node_library} "nav2_longitudinal_state_estimator::LongitudinalStateEstimatorNode")

add_executable(longitudinal_state_estimator
  src/main.cpp
)
ament_target_dependencies(longitudinal_state_estimator rclcpp nav2_util)
target_link_libraries(longitudinal_state_estimator ${node_library})

install(TARGETS
  ${core_library}
  ${node_library}
  longitudinal_state_estimator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS
  scripts/calibrate_chassis_imu_acceleration.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_longitudinal_state_adapter
    test/test_longitudinal_state_adapter.cpp)
  target_link_libraries(test_longitudinal_state_adapter ${core_library})
  ament_target_dependencies(test_longitudinal_state_adapter rclcpp)

  ament_add_gtest(test_lifecycle_bond
    test/test_lifecycle_bond.cpp)
  target_link_libraries(test_lifecycle_bond ${node_library})
  ament_target_dependencies(test_lifecycle_bond rclcpp nav2_util lifecycle_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(${core_library})
ament_export_dependencies(rclcpp nav2_util rosidl_default_runtime)
ament_package()
