cmake_minimum_required(VERSION 3.16)
project(joint_converter)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 设置C++编译器标志
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(end_effector_interfaces REQUIRED)

# 查找 nlohmann_json - 尝试多种方式
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  # 如果通过包管理器找不到，尝试使用CMake的FetchContent或查找本地安装
  find_path(NLOHMANN_JSON_INCLUDE_DIR NAMES nlohmann/json.hpp
    PATHS
    /usr/include
    /usr/local/include
    $ENV{HOME}/.local/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
  )
  if(NLOHMANN_JSON_INCLUDE_DIR)
    set(nlohmann_json_FOUND TRUE)
    message(STATUS "Found nlohmann/json.hpp at: ${NLOHMANN_JSON_INCLUDE_DIR}")
  else()
    message(WARNING "nlohmann_json not found. Please install nlohmann-json3-dev or similar package")
  endif()
endif()

# 包含目录
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${end_effector_interfaces_INCLUDE_DIRS}
)

if(nlohmann_json_FOUND)
  include_directories(${NLOHMANN_JSON_INCLUDE_DIR})
endif()

# 源文件
set(SOURCES
  src/joint_converter.cpp
)

# 创建可执行文件
add_executable(joint_converter ${SOURCES})

# 链接库
ament_target_dependencies(joint_converter
  rclcpp
  std_msgs
  sensor_msgs
  end_effector_interfaces
)

# 链接 nlohmann_json（如果找到）
if(nlohmann_json_FOUND AND NLOHMANN_JSON_INCLUDE_DIR)
  target_include_directories(joint_converter PUBLIC ${NLOHMANN_JSON_INCLUDE_DIR})
elseif(TARGET nlohmann_json::nlohmann_json)
  target_link_libraries(joint_converter nlohmann_json::nlohmann_json)
endif()

# 安装目标
install(TARGETS joint_converter
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件（如果有）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# 安装 config 文件（如果有）
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  end_effector_interfaces
)

if(nlohmann_json_FOUND)
  ament_export_include_directories(${NLOHMANN_JSON_INCLUDE_DIR})
endif()

ament_package()