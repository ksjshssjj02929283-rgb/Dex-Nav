cmake_minimum_required(VERSION 3.8)
project(action_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(fake_nav_server src/fake_nav_server.cpp)
ament_target_dependencies(fake_nav_server rclcpp rclcpp_action nav2_msgs)

add_executable(client_a src/client_a.cpp)
ament_target_dependencies(client_a rclcpp rclcpp_action nav2_msgs)

add_executable(client_b src/client_b.cpp)
ament_target_dependencies(client_b rclcpp rclcpp_action nav2_msgs)

install(TARGETS
  fake_nav_server
  client_a
  client_b
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
