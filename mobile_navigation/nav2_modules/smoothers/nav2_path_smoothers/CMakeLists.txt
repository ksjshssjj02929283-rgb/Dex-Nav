cmake_minimum_required(VERSION 3.8)
project(nav2_path_smoothers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

include_directories(include)

add_library(polynomial_smoother SHARED src/polynomial_smoother.cpp)
ament_target_dependencies(polynomial_smoother ${dependencies})

add_library(elastic_band_smoother SHARED src/elastic_band_smoother.cpp)
ament_target_dependencies(elastic_band_smoother ${dependencies})

add_library(simple_smoother SHARED src/simple_smoother.cpp)
ament_target_dependencies(simple_smoother ${dependencies})

add_library(savitzky_golay_smoother SHARED src/savitzky_golay_smoother.cpp)
ament_target_dependencies(savitzky_golay_smoother ${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS polynomial_smoother elastic_band_smoother simple_smoother savitzky_golay_smoother
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(polynomial_smoother elastic_band_smoother simple_smoother savitzky_golay_smoother)
ament_export_dependencies(${dependencies})
ament_package()
