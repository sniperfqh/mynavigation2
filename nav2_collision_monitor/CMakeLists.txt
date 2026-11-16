cmake_minimum_required(VERSION 3.5)
project(nav2_collision_monitor)

### Dependencies ###

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# ooii 去噪：PCL( RadiusOutlierRemoval 在 filters 模块)
find_package(PCL REQUIRED COMPONENTS common filters)

### Header ###

nav2_package()

### Libraries and executables ###

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_util
  nav2_costmap_2d
  nav2_msgs
  std_msgs
  visualization_msgs
)

set(executable_name collision_monitor)
set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/collision_monitor_node.cpp
  src/polygon.cpp
  src/velocity_polygon.cpp
  src/circle.cpp
  src/exclusion_zone.cpp  #ooii
  src/source.cpp
  src/scan.cpp
  src/pointcloud.cpp
  src/range.cpp
  src/kinematics.cpp
)
# ooii 去噪：链接 PCL(库 + 头文件路径)
target_link_libraries(${library_name} ${PCL_LIBRARIES})
target_include_directories(${library_name} PUBLIC ${PCL_INCLUDE_DIRS})

add_executable(${executable_name}
  src/main.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  ${library_name}
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "nav2_collision_monitor::CollisionMonitor")

### Install ###

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

# 避障状态链：由 launch/obstacle_status.launch.py 以 ros2 run 方式启动。
install(PROGRAMS
  tools/car_op_status_sim.py
  tools/car_op_zones.py
  tools/obstacle_status_translator.py
  DESTINATION lib/${PROJECT_NAME})

# car_op_zones.py 直接读取设计数据，不在脚本里重抄数值。
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../doc/collision_zones/
  DESTINATION share/${PROJECT_NAME}/collision_zones
  FILES_MATCHING PATTERN "gen_zones.py" PATTERN "spec_table*.md")

### Testing ###

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

### Ament stuff ###

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
