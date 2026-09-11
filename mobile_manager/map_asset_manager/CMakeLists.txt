cmake_minimum_required(VERSION 3.8)
project(map_asset_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── 依赖查找 ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

# ── 库目标 ────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME}_lib
  src/data_event_bus.cpp
  src/file_manager.cpp
  src/map_asset_manager_node.cpp
  src/polygon_utils.cpp
  src/spatial_indexer.cpp
  src/point_manager/point_store.cpp
  src/point_manager/nav_point_manager.cpp
  src/route_manager/route_store.cpp
  src/route_manager/route_manager.cpp
  src/area_manager/area_store.cpp
  src/area_manager/area_manager.cpp
  src/room_manager/room_store.cpp
  src/room_manager/room_manager.cpp
  src/route_graph_manager/junction_store.cpp
  src/route_graph_manager/route_edge_store.cpp
  src/route_graph_manager/route_graph_manager.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  std_srvs
  std_msgs
  geometry_msgs
  visualization_msgs
  nav2_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_interfaces
  rcl_interfaces
)

target_link_libraries(${PROJECT_NAME}_lib
  nlohmann_json::nlohmann_json
)

# ── 可执行文件 ────────────────────────────────────────────────────────
add_executable(map_asset_manager_node
  apps/main.cpp
)

target_link_libraries(map_asset_manager_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(map_asset_manager_node
  rclcpp
)

# ── 安装 ──────────────────────────────────────────────────────────────
install(TARGETS
  map_asset_manager_node
  ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/debug/debug_map_asset_manager.sh
  scripts/map_asset_manager_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  test_data/map_assets_standard.json
  DESTINATION share/map_asset_manager/test_data
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # ── 单元测试：PointStore / RouteStore / AreaStore（无 ROS node） ──
  ament_add_gtest(test_stores
    test/test_stores.cpp
  )
  target_include_directories(test_stores PRIVATE include)
  target_link_libraries(test_stores
    ${PROJECT_NAME}_lib
  )
  ament_target_dependencies(test_stores
    rclcpp
    geometry_msgs
  )

  # ── 集成测试：NavPointManager / RouteManager Service 调用 ─────────
  ament_add_gtest(test_services
    test/test_services.cpp
  )
  target_include_directories(test_services PRIVATE include)
  target_link_libraries(test_services
    ${PROJECT_NAME}_lib
  )
  ament_target_dependencies(test_services
    rclcpp
    nav_interfaces
    geometry_msgs
  )
endif()

# ── RViz2 插件（map_asset_rviz_plugin）────────────────────────────────
find_package(rviz_common QUIET)
find_package(rviz_rendering QUIET)
find_package(pluginlib QUIET)
find_package(Qt5 QUIET COMPONENTS Widgets Core)

if(rviz_common_FOUND AND Qt5_FOUND AND pluginlib_FOUND)
  set(CMAKE_AUTOMOC ON)

  add_library(map_asset_rviz_plugin SHARED
    # 源文件
    rviz_plugins/src/map_asset_panel.cpp
    rviz_plugins/src/nav_point_tab.cpp
    rviz_plugins/src/nav_point_draft_bridge.cpp
    rviz_plugins/src/route_tab.cpp
    rviz_plugins/src/area_tab.cpp
    rviz_plugins/src/room_tab.cpp
    rviz_plugins/src/add_nav_point_tool.cpp
    rviz_plugins/src/add_area_tool.cpp
    rviz_plugins/src/add_room_tool.cpp
    rviz_plugins/src/route_graph_tab.cpp
    rviz_plugins/src/add_junction_tool.cpp
    rviz_plugins/src/add_edge_tool.cpp
    rviz_plugins/src/add_curve_edge_tool.cpp
    rviz_plugins/src/route_graph_delete_tool.cpp
    rviz_plugins/src/route_graph_start_save_tool.cpp
    # 头文件须显式列出，否则 AUTOMOC 不会扫描独立 include 目录下的 Q_OBJECT 宏
    rviz_plugins/include/map_asset_rviz_plugin/map_asset_panel.hpp
    rviz_plugins/include/map_asset_rviz_plugin/nav_point_tab.hpp
    rviz_plugins/include/map_asset_rviz_plugin/nav_point_draft_bridge.hpp
    rviz_plugins/include/map_asset_rviz_plugin/route_tab.hpp
    rviz_plugins/include/map_asset_rviz_plugin/area_tab.hpp
    rviz_plugins/include/map_asset_rviz_plugin/room_tab.hpp
    rviz_plugins/include/map_asset_rviz_plugin/add_nav_point_tool.hpp
    rviz_plugins/include/map_asset_rviz_plugin/add_area_tool.hpp
    rviz_plugins/include/map_asset_rviz_plugin/add_room_tool.hpp
    rviz_plugins/include/map_asset_rviz_plugin/route_graph_tab.hpp
    rviz_plugins/include/map_asset_rviz_plugin/add_junction_tool.hpp
    rviz_plugins/include/map_asset_rviz_plugin/add_edge_tool.hpp
    rviz_plugins/include/map_asset_rviz_plugin/add_curve_edge_tool.hpp
    rviz_plugins/include/map_asset_rviz_plugin/route_graph_delete_tool.hpp
    rviz_plugins/include/map_asset_rviz_plugin/route_graph_start_save_tool.hpp
  )

  target_include_directories(map_asset_rviz_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugins/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(map_asset_rviz_plugin
    rclcpp
    rclcpp_action
    nav_interfaces
    geometry_msgs
    std_srvs
    rviz_common
    rviz_rendering
    pluginlib
  )

  target_link_libraries(map_asset_rviz_plugin
    Qt5::Widgets
    Qt5::Core
  )

  pluginlib_export_plugin_description_file(rviz_common rviz_plugins/plugin_description.xml)

  install(TARGETS map_asset_rviz_plugin
    EXPORT export_map_asset_rviz_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY rviz_plugins/include/
    DESTINATION include
  )

  ament_export_targets(export_map_asset_rviz_plugin HAS_LIBRARY_TARGET)
  ament_export_dependencies(rviz_common rviz_rendering pluginlib)
endif()

ament_package()
