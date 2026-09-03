cmake_minimum_required(VERSION 3.8)
project(w1_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(sim_interface_contract test/test_sim_interface_contract.py)
endif()

install(DIRECTORY
  maps
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# 仿真运行时轻量节点：路径可视化与底盘就绪检测。
install(PROGRAMS
  script/path_trace_visualizer.py
  script/wait_for_gazebo_ready.py
  script/wait_for_sim_ready.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
