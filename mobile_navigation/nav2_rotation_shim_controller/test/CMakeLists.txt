# tests
find_package(nav2_controller REQUIRED)
find_package(simple_goal_checker REQUIRED)

ament_add_gtest(test_shim_controller
  test_shim_controller.cpp
)
ament_target_dependencies(test_shim_controller
  ${dependencies}
  nav2_controller
  simple_goal_checker
)
target_link_libraries(test_shim_controller
  ${library_name}
)
