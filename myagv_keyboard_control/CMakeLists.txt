cmake_minimum_required(VERSION 3.8)
project(myagv_keyboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(byd_custom_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(myagv_keyboard_control_node src/myagv_keyboard_control_node.cpp)
ament_target_dependencies(
  myagv_keyboard_control_node
  byd_custom_msgs
  rclcpp
)

target_compile_features(myagv_keyboard_control_node PUBLIC cxx_std_17)

install(
  TARGETS myagv_keyboard_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
