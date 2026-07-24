# Test dynamic parameters
ament_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
ament_target_dependencies(test_dynamic_parameters
  ${dependencies}
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
)

ament_add_gtest(test_local_planning_stage
  test_local_planning_stage.cpp
)
ament_target_dependencies(test_local_planning_stage
  ${dependencies}
)
target_link_libraries(test_local_planning_stage
  ${library_name}
)

ament_add_gtest(near_goal_odom_context_test
  near_goal_odom_context_test.cpp
)
target_include_directories(near_goal_odom_context_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
ament_target_dependencies(near_goal_odom_context_test
  ${dependencies}
)
target_link_libraries(near_goal_odom_context_test
  ${library_name}
)
