cmake_minimum_required(VERSION 3.8)
project(mission_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)

find_package(glog REQUIRED)
find_package(PkgConfig REQUIRED)

include_directories(
  include
  include/mission_manager
  apps
  ${GLOG_INCLUDE_DIRS}
)

set(THIRD_PARTY_LIBS
  glog::glog
)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  std_srvs
  std_msgs
  nav_interfaces
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  nav2_util
)

# Mission manager library
add_library(mission_manager SHARED
  src/mission_manager.cpp
  src/tasks/task_navigation.cpp
  src/tasks/task_docking_pile.cpp
  src/tasks/task_exit_pile.cpp
  src/tasks/task_auto_charging.cpp
)

ament_target_dependencies(mission_manager ${dependencies})
target_link_libraries(mission_manager ${THIRD_PARTY_LIBS})

# Mission manager node executable
add_executable(mission_manager_node
  apps/mission_manager_node.cpp
  apps/mission_manager_ros.cpp
)

ament_target_dependencies(mission_manager_node ${dependencies})
target_link_libraries(mission_manager_node mission_manager ${THIRD_PARTY_LIBS})

# Install targets
install(TARGETS mission_manager_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(pub_current_position src/pub_current_position.cpp)
ament_target_dependencies(pub_current_position rclcpp tf2_ros geometry_msgs)
install(TARGETS pub_current_position DESTINATION lib/${PROJECT_NAME})


install(TARGETS mission_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/mission_manager/
  DESTINATION include/${PROJECT_NAME}/mission_manager
  FILES_MATCHING PATTERN "*.hpp"
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install params files
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(mission_manager)
ament_export_dependencies(${dependencies})

ament_package()

