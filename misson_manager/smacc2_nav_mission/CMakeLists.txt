cmake_minimum_required(VERSION 3.8)
project(smacc2_nav_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(smacc2 REQUIRED)
find_package(cl_nav2z REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(
  include
  ${smacc2_INCLUDE_DIRS}
  ${cl_nav2z_INCLUDE_DIRS}
  ${nav_interfaces_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/smacc2_nav_mission/smacc2_nav_mission_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${smacc2_LIBRARIES}
  ${cl_nav2z_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  smacc2
  cl_nav2z
  nav_interfaces
  geometry_msgs
  tf2
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

ament_package()
