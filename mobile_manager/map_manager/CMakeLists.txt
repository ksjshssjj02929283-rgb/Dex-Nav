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

install(PROGRAMS
  scripts/map_manager_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  test_support/build_standard_map_data.py
  test_support/validate_standard_map_data.py
  test_support/map_manager_auto_test.py
  test_support/map_manager_data_test.py
  test_support/run_map_manager_acceptance.py
  test_support/run_map_manager_local_self_test.py
  test_support/map_manager_test_paths.py
  DESTINATION lib/${PROJECT_NAME}/map_manager_test_support
)

install(FILES
  test_data/standard_map_manifest.json
  DESTINATION share/${PROJECT_NAME}/test_data
)

install(DIRECTORY
  test_data/map_standard
  DESTINATION share/${PROJECT_NAME}/test_data
)

# Independent NodeMap RViz editor. The plugin reuses nav2_map_server's
# NodeMap file interfaces without changing map_manager_node behavior.
find_package(rviz_common QUIET)
find_package(rviz_rendering QUIET)
find_package(pluginlib QUIET)
find_package(Qt5 QUIET COMPONENTS Widgets Core)
find_package(nav2_map_server QUIET)
find_package(visualization_msgs QUIET)

if(
  rviz_common_FOUND AND rviz_rendering_FOUND AND pluginlib_FOUND AND
  Qt5_FOUND AND nav2_map_server_FOUND AND visualization_msgs_FOUND)
  set(CMAKE_AUTOMOC ON)

  add_library(nodemap_rviz_plugin SHARED
    nodemap_rviz_plugin/src/editor_model.cpp
    nodemap_rviz_plugin/src/nodemap_editor_panel.cpp
    nodemap_rviz_plugin/src/move_node_tool.cpp
    nodemap_rviz_plugin/src/edit_edge_tool.cpp
    nodemap_rviz_plugin/include/nodemap_rviz_plugin/editor_model.hpp
    nodemap_rviz_plugin/include/nodemap_rviz_plugin/nodemap_editor_panel.hpp
    nodemap_rviz_plugin/include/nodemap_rviz_plugin/move_node_tool.hpp
    nodemap_rviz_plugin/include/nodemap_rviz_plugin/edit_edge_tool.hpp
  )

  target_compile_features(nodemap_rviz_plugin PUBLIC cxx_std_17)
  target_include_directories(nodemap_rviz_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/nodemap_rviz_plugin/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(nodemap_rviz_plugin
    rclcpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    nav2_map_server
    rviz_common
    rviz_rendering
    pluginlib
  )
  target_link_libraries(nodemap_rviz_plugin
    Qt5::Widgets
    Qt5::Core
  )

  pluginlib_export_plugin_description_file(
    rviz_common nodemap_rviz_plugin/plugin_description.xml)

  install(TARGETS nodemap_rviz_plugin
    EXPORT export_nodemap_rviz_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(DIRECTORY nodemap_rviz_plugin/include/
    DESTINATION include
  )
  install(FILES nodemap_rviz_plugin/README.md
    DESTINATION share/${PROJECT_NAME}/nodemap_rviz_plugin
  )

  ament_export_targets(export_nodemap_rviz_plugin HAS_LIBRARY_TARGET)
  ament_export_dependencies(
    rviz_common rviz_rendering pluginlib nav2_map_server visualization_msgs)
endif()

ament_package()
