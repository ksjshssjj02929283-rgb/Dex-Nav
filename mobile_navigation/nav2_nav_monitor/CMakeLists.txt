cmake_minimum_required(VERSION 3.5)
project(nav2_nav_monitor)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

nav2_package()

include_directories(
  include
)

set(executable_name nav_monitor)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  action_msgs
)

add_library(${library_name} SHARED
  src/nav_monitor_node.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${executable_name}
  src/main.cpp
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)
target_link_libraries(${executable_name} ${library_name})

install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/plot_nav_task_path.py
  scripts/plot_nav_task_speed.py
  scripts/analyze_nav_tasks.py
  scripts/gen_test_nav_tasks.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/nav_task_common.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
