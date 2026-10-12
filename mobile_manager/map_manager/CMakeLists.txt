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
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

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
  src/map_yaml_doc.cpp
  src/grid_edit.cpp
)

ament_target_dependencies(map_manager_core
  rclcpp
  nav_msgs
  tf2
  ament_index_cpp
)

target_include_directories(map_manager_core PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(map_manager_core
  Boost::filesystem
  ${OpenCV_LIBS}
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

# 地图编辑核心层测试（standalone，运行：`ros2 run map_manager map_manager_edit_test`）
add_executable(map_manager_edit_test
  test_support/map_manager_edit_test.cpp
)
target_link_libraries(map_manager_edit_test
  map_manager_core
)

install(TARGETS
  map_manager_core
  map_manager_node
  map_manager_edit_test
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

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_manager_test.py
  scripts/map_edit_rviz_helper.py
  test_support/run_map_edit_self_test.py
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

# ── RViz2 插件（map_manager_rviz_plugin）：无 RViz 环境（如机器人本体）时自动跳过 ──────
find_package(rviz_common QUIET)
find_package(rviz_rendering QUIET)
find_package(pluginlib QUIET)
find_package(Qt5 QUIET COMPONENTS Widgets Core)

if(rviz_common_FOUND AND rviz_rendering_FOUND AND Qt5_FOUND AND pluginlib_FOUND)
  set(CMAKE_AUTOMOC ON)

  add_library(map_manager_rviz_plugin SHARED
    rviz_plugins/src/grid_edit_draft.cpp
    rviz_plugins/src/map_edit_tool.cpp
    rviz_plugins/src/map_manager_panel.cpp
    # 头文件须显式列出，否则 AUTOMOC 不会扫描独立 include 目录下的 Q_OBJECT 宏
    rviz_plugins/include/map_manager_rviz_plugin/grid_edit_draft.hpp
    rviz_plugins/include/map_manager_rviz_plugin/map_edit_tool.hpp
    rviz_plugins/include/map_manager_rviz_plugin/map_manager_panel.hpp
  )
  target_include_directories(map_manager_rviz_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugins/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(map_manager_rviz_plugin
    rclcpp
    nav_interfaces
    geometry_msgs
    rviz_common
    rviz_rendering
    pluginlib
  )
  target_link_libraries(map_manager_rviz_plugin Qt5::Widgets Qt5::Core)

  pluginlib_export_plugin_description_file(rviz_common rviz_plugins/plugin_description.xml)

  install(TARGETS map_manager_rviz_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
else()
  message(STATUS "rviz_common / Qt5 not found, skip map_manager_rviz_plugin")
endif()

ament_package()
