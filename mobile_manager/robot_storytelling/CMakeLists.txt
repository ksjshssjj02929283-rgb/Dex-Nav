cmake_minimum_required(VERSION 3.5)
project(robot_storytelling)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  nav_interfaces
)

# 所有 teller 编译进同一个共享库：teller 只通过 TellerRegistry 的运行时字符串
# 查找被使用（storytelling_node.cpp 不直接引用 AreaAttributeTeller /
# RouteAttributeTeller 符号），若把某个 teller 单独编成静态库、又没有代码
# 直接引用其符号，链接器可能整个丢弃该编译单元，ROBOT_STORYTELLER_REGISTER
# 的静态注册对象就不会执行。同库编译规避这个问题。
add_library(storytelling_core SHARED
  src/teller_registry.cpp
  src/area_attribute_teller.cpp
  src/route_attribute_teller.cpp
  src/storytelling_node.cpp
)
ament_target_dependencies(storytelling_core ${dependencies})

install(TARGETS storytelling_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

add_executable(storytelling_node src/main.cpp)
target_link_libraries(storytelling_node storytelling_core)
ament_target_dependencies(storytelling_node ${dependencies})

install(TARGETS storytelling_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_area_attribute_teller test/test_area_attribute_teller.cpp)
  target_link_libraries(test_area_attribute_teller storytelling_core)
  ament_target_dependencies(test_area_attribute_teller ${dependencies})

  ament_add_gtest(test_route_attribute_teller test/test_route_attribute_teller.cpp)
  target_link_libraries(test_route_attribute_teller storytelling_core)
  ament_target_dependencies(test_route_attribute_teller ${dependencies})
endif()

ament_export_libraries(storytelling_core)
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_package()
