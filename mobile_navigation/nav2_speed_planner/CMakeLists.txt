cmake_minimum_required(VERSION 3.5)
project(nav2_speed_planner)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ruckig REQUIRED)  # jerk 限制轨迹生成（/usr/local，S 型速度曲线）

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(dependencies
  angles
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  nav_interfaces
  nav2_costmap_2d
  nav2_util
  nav2_core
  pluginlib
  tf2_ros
)

set(library_name nav2_speed_planner)
set(support_library_name nav2_speed_plan_state)

# SpeedPlanState is linked by controller_server. Keep it outside the plugin
# library so loading controller_server cannot preload the plugin factories
# before pluginlib creates its ClassLoader.
add_library(${support_library_name} SHARED
  src/speed_plan_state.cpp)

ament_target_dependencies(${support_library_name}
  ${dependencies}
)

# pluginlib-only library: it must only be opened by pluginlib.
add_library(${library_name} SHARED
  src/speed_planner_base.cpp
  src/trapezoidal_speed_planner.cpp
  src/scurve_speed_planner.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name} ruckig::ruckig)

add_executable(speed_profile_gen src/tools/speed_profile_gen.cpp)
target_link_libraries(speed_profile_gen ${library_name} ruckig::ruckig)
ament_target_dependencies(speed_profile_gen ${dependencies})

install(TARGETS speed_profile_gen
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/plot_speed_profiles.py
  scripts/gen_speed_plan_charts.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${support_library_name} ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # S 型速度曲线（Ruckig）纯算法单测（jerk≤max_jerk / 加速度有界 / 不超包络 / 停车）。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_speed_plan_state test/test_speed_plan_state.cpp)
  target_link_libraries(test_speed_plan_state ${support_library_name})
  ament_target_dependencies(test_speed_plan_state ${dependencies})

  ament_add_gtest(test_scurve_profile test/test_scurve_profile.cpp)
  target_link_libraries(test_scurve_profile ${library_name} ruckig::ruckig)
  ament_target_dependencies(test_scurve_profile ${dependencies})

  # T/S 端到端剖面测试（0.3/0.5/0.75/1.0m 直线，静止起步；导出 CSV 供画图）。
  ament_add_gtest(test_speed_profiles test/test_speed_profiles.cpp)
  target_link_libraries(test_speed_profiles ${library_name} ruckig::ruckig)
  ament_target_dependencies(test_speed_profiles ${dependencies})
endif()

ament_export_include_directories(include)
# Consumers only need the non-plugin state library. Exporting the plugin
# library would make component containers preload it and lose its factories.
ament_export_libraries(${support_library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2_speed_planner.xml)

ament_package()
