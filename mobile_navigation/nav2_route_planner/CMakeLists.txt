cmake_minimum_required(VERSION 3.5)
project(nav2_route_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(nav2_theta_star_planner REQUIRED)
find_package(visualization_msgs REQUIRED)

nav2_package()
add_compile_options(-O3)

include_directories(include)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  geometry_msgs
  nav_msgs
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  nav_interfaces
  nav2_theta_star_planner
  visualization_msgs
)

add_library(${library_name} SHARED
  src/topo_graph.cpp
  src/route_graph_loader.cpp
  src/start_goal_projector.cpp
  src/graph_augmenter.cpp
  src/cost_function.cpp
  src/astar_searcher.cpp
  src/path_interpolator.cpp
  src/route_plan_publisher.cpp
  src/costmap_aware_layer.cpp
  src/nav_route_graph_planner.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB_DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core nav2_route_planner.xml)

add_executable(nav_route_native_planner src/native_planner_main.cpp)
ament_target_dependencies(nav_route_native_planner ${dependencies})
target_link_libraries(nav_route_native_planner ${library_name})

install(TARGETS ${library_name} nav_route_native_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES nav2_route_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(gtest_disable_pthreads OFF)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_topo_graph test/test_topo_graph.cpp)
  ament_target_dependencies(test_topo_graph ${dependencies})
  target_link_libraries(test_topo_graph ${library_name})

  ament_add_gtest(test_start_goal_projector test/test_start_goal_projector.cpp)
  ament_target_dependencies(test_start_goal_projector ${dependencies})
  target_link_libraries(test_start_goal_projector ${library_name})

  ament_add_gtest(test_astar_searcher test/test_astar_searcher.cpp)
  ament_target_dependencies(test_astar_searcher ${dependencies})
  target_link_libraries(test_astar_searcher ${library_name})

  ament_add_gtest(test_graph_augmenter test/test_graph_augmenter.cpp)
  ament_target_dependencies(test_graph_augmenter ${dependencies})
  target_link_libraries(test_graph_augmenter ${library_name})

  ament_add_gtest(test_path_interpolator test/test_path_interpolator.cpp)
  ament_target_dependencies(test_path_interpolator ${dependencies})
  target_link_libraries(test_path_interpolator ${library_name})

  ament_add_gtest(test_costmap_aware_layer test/test_costmap_aware_layer.cpp)
  ament_target_dependencies(test_costmap_aware_layer ${dependencies})
  target_link_libraries(test_costmap_aware_layer ${library_name})
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
