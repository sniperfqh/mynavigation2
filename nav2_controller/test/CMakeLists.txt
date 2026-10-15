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
ament_add_gtest(test_collision_progress test_collision_progress.cpp)
ament_target_dependencies(test_collision_progress ${dependencies})
target_link_libraries(test_collision_progress ${library_name} simple_progress_checker)
