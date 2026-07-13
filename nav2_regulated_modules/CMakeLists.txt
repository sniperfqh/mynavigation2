cmake_minimum_required(VERSION 3.8)
project(nav2_regulated_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(regulated_navigator_node src/regulated_navigator_node.cpp)
target_include_directories(regulated_navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(regulated_navigator_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(regulated_navigator_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
)

install(TARGETS regulated_navigator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include
  DESTINATION include
)

install(DIRECTORY
  launch
  maps
  params
  rviz
  src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
