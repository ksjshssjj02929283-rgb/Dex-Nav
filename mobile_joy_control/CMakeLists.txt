cmake_minimum_required(VERSION 3.5)
project(mobile_joy_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(chassis_interfaces REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(supervisor_interfaces REQUIRED)

# Find SDL2
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(SDL2 QUIET sdl2)
endif()

# 如果pkg-config找不到，尝试使用find_package
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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stop_command_handler test/test_stop_command_handler.cpp)
  if(TARGET test_stop_command_handler)
    target_include_directories(test_stop_command_handler PRIVATE include)
  endif()
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(mobile_joy_control
  src/mobile_joy_control.cpp
)
target_include_directories(mobile_joy_control PRIVATE include)
target_compile_features(mobile_joy_control PRIVATE cxx_std_17)

# 🔧 STEERING_ACTION_DIRECT: mobile_joy_control keeps the Action client only; Server is in ChassisNode.
add_executable(omni_start_coordinator
  src/omni_start_coordinator.cpp
)
target_compile_features(omni_start_coordinator PRIVATE cxx_std_17)

ament_target_dependencies(mobile_joy_control
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  supervisor_interfaces
)

if(SDL2_FOUND)
  target_include_directories(mobile_joy_control PRIVATE ${SDL2_INCLUDE_DIRS})
  target_link_libraries(mobile_joy_control ${SDL2_LIBRARIES})
  if(SDL2_CFLAGS_OTHER)
    target_compile_options(mobile_joy_control PRIVATE ${SDL2_CFLAGS_OTHER})
  endif()
  target_compile_definitions(mobile_joy_control PRIVATE HAVE_SDL2)
endif()

ament_target_dependencies(omni_start_coordinator
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  std_msgs
  chassis_interfaces
  nav_interfaces
)

install(TARGETS
  mobile_joy_control
  omni_start_coordinator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)
# 安装配置文件
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
