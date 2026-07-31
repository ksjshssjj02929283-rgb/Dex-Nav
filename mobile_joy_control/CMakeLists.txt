cmake_minimum_required(VERSION 3.5)
project(mobile_joy_control)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(chassis_interfaces REQUIRED)
find_package(nav_interfaces REQUIRED)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(SDL2 QUIET sdl2)
endif()

if(NOT SDL2_FOUND)
  find_path(SDL2_INCLUDE_DIRS SDL.h
    PATHS
    /usr/include/SDL2
    /usr/local/include/SDL2
    /opt/local/include/SDL2
  )
  find_library(SDL2_LIBRARIES
    NAMES SDL2
    PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
  )
  if(SDL2_INCLUDE_DIRS AND SDL2_LIBRARIES)
    set(SDL2_FOUND TRUE)
    set(SDL2_CFLAGS_OTHER "")
  endif()
endif()

if(NOT SDL2_FOUND)
  message(WARNING "SDL2 not found. Joystick name detection will use default Logitech parameters.")
  set(SDL2_INCLUDE_DIRS "")
  set(SDL2_LIBRARIES "")
  set(SDL2_CFLAGS_OTHER "")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# CMake target 使用独立名称，OUTPUT_NAME 仍为 mobile_joy_control，
# launch 里的 executable 不需要修改。
add_executable(mobile_joy_control_node
  src/mobile_joy_control.cpp
)
set_target_properties(mobile_joy_control_node PROPERTIES OUTPUT_NAME "mobile_joy_control")

add_executable(mobile_cmd_omni
  src/mobile_cmd_omni.cpp
)

ament_target_dependencies(mobile_joy_control_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

if(SDL2_FOUND)
  target_include_directories(mobile_joy_control_node PRIVATE ${SDL2_INCLUDE_DIRS})
  target_link_libraries(mobile_joy_control_node ${SDL2_LIBRARIES})
  if(SDL2_CFLAGS_OTHER)
    target_compile_options(mobile_joy_control_node PRIVATE ${SDL2_CFLAGS_OTHER})
  endif()
  target_compile_definitions(mobile_joy_control_node PRIVATE HAVE_SDL2)
endif()

ament_target_dependencies(mobile_cmd_omni
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  chassis_interfaces
  nav_interfaces
)

install(TARGETS
  mobile_joy_control_node
  mobile_cmd_omni
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/omni_chassis_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
