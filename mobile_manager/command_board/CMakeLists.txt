cmake_minimum_required(VERSION 3.10)
project(command_board)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PkgConfig REQUIRED)
find_package(glog REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
  include/fsm
)


add_library(command_board_core
  src/fsm/localization_state_sm.cpp
)

ament_target_dependencies(command_board_core
  nav2_msgs
  action_msgs
  rclcpp_action
  tf2
)

target_link_libraries(command_board_core
  Boost::filesystem
  glog::glog
  yaml-cpp
)

# 导航门面：独立共享库 + rclcpp_components 组件。宿主可选：command_board 进程（当前，下面直接链接）、
# 独立可执行 navigation_node（组件宏自动生成）、或任意容器 / mission_manager 进程按组件加载；接口名不随宿主变。
add_library(navigation_facade SHARED
  src/navigation_facade.cpp
)
ament_target_dependencies(navigation_facade
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  diagnostic_msgs
  std_msgs
  nav_interfaces
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)
rclcpp_components_register_node(navigation_facade
  PLUGIN "command_board::NavigationFacade"
  EXECUTABLE navigation_node
)

add_executable(command_board_node
  apps/command_board_node.cpp
  apps/command_board_ros.cpp
  src/map_version_manager.cpp
  src/status_monitor.cpp
  src/sensor_monitor.cpp
  src/auto_map_saver.cpp
)

ament_target_dependencies(command_board_node
  rclcpp
  nav2_msgs
  action_msgs
  rclcpp_action
  diagnostic_msgs
  std_srvs
  std_msgs
  nav_interfaces
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(command_board_node
  command_board_core
  navigation_facade
  nlohmann_json::nlohmann_json
)

install(TARGETS navigation_facade
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  command_board_core
  command_board_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# ── 单元测试（SensorMonitor）──────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(sensor_monitor_test
    test/sensor_monitor_test.cpp
    src/sensor_monitor.cpp)

  ament_target_dependencies(sensor_monitor_test
    rclcpp
    nav_interfaces
    nav_msgs
    sensor_msgs)

  target_include_directories(sensor_monitor_test PRIVATE include)
endif()

# ── RViz2 状态显示面板（command_board_rviz_plugins）──────────────────
# 订阅 ~/nav_system_status + ~/auto_map_saver_status，分区卡片可视化。
# 无 RViz 环境时 QUIET 跳过，不阻塞 command_board_node 构建。
find_package(rviz_common QUIET)
find_package(rviz_rendering QUIET)
find_package(pluginlib QUIET)
find_package(Qt5 QUIET COMPONENTS Widgets Core)

if(rviz_common_FOUND AND Qt5_FOUND AND pluginlib_FOUND)
  set(CMAKE_AUTOMOC ON)

  add_library(command_board_rviz_plugins SHARED
    rviz_plugins/src/command_board_status_panel.cpp
    rviz_plugins/src/command_board_control_panel.cpp
    # 含 Q_OBJECT 的头文件须显式列出，否则 AUTOMOC 不扫描独立 include 目录
    rviz_plugins/include/command_board_rviz_plugins/command_board_status_panel.hpp
    rviz_plugins/include/command_board_rviz_plugins/command_board_control_panel.hpp)

  target_include_directories(command_board_rviz_plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugins/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(command_board_rviz_plugins
    rclcpp
    nav_interfaces
    geometry_msgs
    std_msgs
    std_srvs
    rviz_common
    rviz_rendering
    pluginlib)

  target_link_libraries(command_board_rviz_plugins
    Qt5::Widgets
    Qt5::Core)

  pluginlib_export_plugin_description_file(rviz_common rviz_plugins/plugin_description.xml)

  install(TARGETS command_board_rviz_plugins
    EXPORT export_command_board_rviz_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  install(DIRECTORY rviz_plugins/include/
    DESTINATION include)

  ament_export_targets(export_command_board_rviz_plugins HAS_LIBRARY_TARGET)
  ament_export_dependencies(rviz_common rviz_rendering pluginlib)
endif()

ament_package()
