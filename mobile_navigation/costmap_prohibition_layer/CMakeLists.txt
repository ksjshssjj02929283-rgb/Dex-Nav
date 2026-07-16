cmake_minimum_required(VERSION 3.5)
project(costmap_prohibition_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  pluginlib
  nav2_costmap_2d
  nav_interfaces
  geometry_msgs
)

add_library(prohibition_layer SHARED
  src/prohibition_layer.cpp
)
ament_target_dependencies(prohibition_layer ${dependencies})

install(TARGETS prohibition_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_prohibition_layer test/test_prohibition_layer.cpp)
  target_link_libraries(test_prohibition_layer prohibition_layer)
  ament_target_dependencies(test_prohibition_layer ${dependencies})
endif()

ament_export_libraries(prohibition_layer)
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

ament_package()
