cmake_minimum_required(VERSION 3.8)
project(nav_benchmark_rviz)

find_package(ament_cmake REQUIRED)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rviz_common QUIET)
find_package(pluginlib QUIET)
find_package(Qt5 QUIET COMPONENTS Widgets Core)

# 没有 RViz / Qt 的环境（如部分 CI）跳过面板，不影响整个工作区编译
if(rclcpp_FOUND AND std_msgs_FOUND AND rviz_common_FOUND AND pluginlib_FOUND AND Qt5_FOUND)
  set(CMAKE_AUTOMOC ON)
  add_library(${PROJECT_NAME} SHARED
    src/benchmark_panel.cpp
    # 头文件显式列出，AUTOMOC 才会扫描 include 目录里的 Q_OBJECT
    include/nav_benchmark_rviz/benchmark_panel.hpp)
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
  ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs rviz_common pluginlib)
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core)
  pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  install(DIRECTORY include/ DESTINATION include)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(rclcpp std_msgs rviz_common pluginlib)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_benchmark_panel test/test_benchmark_panel.cpp
      SKIP_LINKING_MAIN_LIBRARIES
      ENV QT_QPA_PLATFORM=offscreen)
    target_link_libraries(test_benchmark_panel ${PROJECT_NAME} Qt5::Widgets gtest)
    ament_target_dependencies(test_benchmark_panel rviz_common pluginlib)
    target_compile_definitions(test_benchmark_panel PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endif()
else()
  message(STATUS "nav_benchmark_rviz: rviz_common / pluginlib / Qt5 not found, panel skipped")
endif()

ament_package()
