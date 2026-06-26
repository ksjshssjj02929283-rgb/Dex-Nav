cmake_minimum_required(VERSION 3.8)
project(dexforce_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装URDF和mesh文件
install(
        DIRECTORY urdf meshes
        DESTINATION share/${PROJECT_NAME}
)

# 安装launch文件
install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()