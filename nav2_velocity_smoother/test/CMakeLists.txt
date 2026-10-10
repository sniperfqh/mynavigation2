# Tests
ament_add_gtest(velocity_smoother_tests
  test_velocity_smoother.cpp
)
ament_target_dependencies(velocity_smoother_tests
  ${dependencies}
)
target_link_libraries(velocity_smoother_tests
  ${library_name}
)

add_executable(jerk_profile_test jerk_profile_test.cpp)
target_compile_features(jerk_profile_test PRIVATE cxx_std_17)
add_test(NAME jerk_profile_test COMMAND jerk_profile_test)
