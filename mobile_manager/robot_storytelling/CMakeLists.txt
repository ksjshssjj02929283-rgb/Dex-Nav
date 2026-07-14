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

add_library(story_evaluator SHARED
  src/story_evaluator.cpp
)
ament_target_dependencies(story_evaluator ${dependencies})

install(TARGETS story_evaluator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

add_library(storytelling_core SHARED
  src/storytelling_node.cpp
)
target_link_libraries(storytelling_core story_evaluator)
ament_target_dependencies(storytelling_core ${dependencies})

add_executable(storytelling_node src/main.cpp)
target_link_libraries(storytelling_node storytelling_core)
ament_target_dependencies(storytelling_node ${dependencies})

install(TARGETS storytelling_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS storytelling_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_story_evaluator test/test_story_evaluator.cpp)
  target_link_libraries(test_story_evaluator story_evaluator)
  ament_target_dependencies(test_story_evaluator ${dependencies})
endif()

ament_export_libraries(story_evaluator)
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_package()
