# 构建结构：协调器进程、固定路径插件、诊断和边界可视化分别生成目标。
# 头文件、Launch 和资源安装到各自前缀；BUILD_TESTING 控制回归目标，不代表测试在构建时自动运行。
cmake_minimum_required(VERSION 3.8)
project(nav2_regulated_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖在配置阶段发现；这里的 REQUIRED 失败是构建环境问题，不是运行时导航结果。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(byd_custom_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(spdlog_wrapper REQUIRED)
find_package(visualization_msgs REQUIRED)

# 协调器进程将按职责拆分的 navigator_* 文件链接在一起，共享同一 RegulatedNavigator 实例状态。
add_executable(regulated_navigator_node
  src/regulated_navigator_node.cpp
  src/regulated_navigator.cpp
  src/chassis_control_subscriber.cpp
  src/motion_state_subscriber.cpp
  src/navigator_goal_handler.cpp
  src/navigator_planning.cpp
  src/navigator_fixed_path.cpp
  src/navigator_control.cpp
  src/navigator_monitor.cpp
  src/navigator_task_manager.cpp
  src/navigation_utils.cpp
  src/planning_module.cpp
  src/control_module.cpp)
target_include_directories(regulated_navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(regulated_navigator_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(regulated_navigator_node
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  nav2_msgs
  nav2_util
  std_msgs
  nav_msgs
  byd_custom_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  spdlog_wrapper
  visualization_msgs
)

# Controller 与 GoalChecker 编译为同一插件共享库，由 pluginlib 根据 XML 导出声明加载。
add_library(nav2_regulated_modules_fixed_path_controller SHARED
  src/fixed_path_controller.cpp
  src/fixed_path_goal_checker.cpp)
target_include_directories(nav2_regulated_modules_fixed_path_controller PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(nav2_regulated_modules_fixed_path_controller PUBLIC cxx_std_17)
ament_target_dependencies(nav2_regulated_modules_fixed_path_controller rclcpp rclcpp_lifecycle geometry_msgs nav2_core nav2_costmap_2d nav2_util nav_msgs pluginlib tf2 tf2_ros tf2_geometry_msgs spdlog_wrapper)

# 诊断进程独立于控制周期；边界可视化也只发布显示消息，不进入速度计算。
add_executable(velocity_diagnostics_node src/velocity_diagnostics.cpp)
target_compile_features(velocity_diagnostics_node PUBLIC cxx_std_17)
ament_target_dependencies(velocity_diagnostics_node rclcpp geometry_msgs nav_msgs byd_custom_msgs spdlog_wrapper)

add_executable(collision_boundary_visualizer_node src/collision_boundary_visualizer.cpp)
target_compile_features(collision_boundary_visualizer_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(collision_boundary_visualizer_node rclcpp geometry_msgs visualization_msgs)

install(TARGETS nav2_regulated_modules_fixed_path_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS regulated_navigator_node collision_boundary_visualizer_node velocity_diagnostics_node
  DESTINATION lib/${PROJECT_NAME})

# 安装公开头文件及运行资源；symlink-install 下部分安装路径依赖 build 或源目录，不可直接整目录清理。
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

pluginlib_export_plugin_description_file(nav2_core fixed_path_controller_plugin.xml)
ament_export_include_directories(include)
ament_export_libraries(nav2_regulated_modules_fixed_path_controller)
ament_export_dependencies(nav2_core nav2_costmap_2d pluginlib)

include(CTest)
# 将 collision 测试客户端安装为 ros2 run 可执行脚本；安装不代表它会随 Launch 自动运行。
install(PROGRAMS tools/collision_action_test.py DESTINATION lib/${PROJECT_NAME})
# 以下注册纯算法／Launch／窗口测试；CTest 执行与 colcon 构建是两个步骤，未执行不能宣称测试通过。
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME collision_action_tool_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_collision_action_tool.py)
  add_test(NAME logging_routing_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_logging_routing.py)
  add_test(NAME velocity_logging_launch_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/velocity_logging_launch_test.py)
  add_test(NAME collision_launch_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_collision_launch.py)
  add_executable(terminal_convergence_test test/terminal_convergence_test.cpp)
  target_include_directories(terminal_convergence_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(terminal_convergence_test nav2_core)
  add_test(NAME terminal_convergence_test COMMAND terminal_convergence_test)
  add_executable(braking_margin_test test/braking_margin_test.cpp)
  target_include_directories(braking_margin_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(braking_margin_test PRIVATE cxx_std_17)
  add_test(NAME braking_margin_test COMMAND braking_margin_test)
  add_executable(terminal_position_test test/terminal_position_test.cpp)
  target_include_directories(terminal_position_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(terminal_position_test PRIVATE cxx_std_17)
  add_test(NAME terminal_position_test COMMAND terminal_position_test)
  add_executable(path_sample_test test/path_sample_test.cpp)
  target_include_directories(path_sample_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(path_sample_test PRIVATE cxx_std_17)
  add_test(NAME path_sample_test COMMAND path_sample_test)
  add_executable(fixed_path_start_speed_guard_test test/fixed_path_start_speed_guard_test.cpp)
  target_include_directories(fixed_path_start_speed_guard_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(fixed_path_start_speed_guard_test PRIVATE cxx_std_17)
  add_test(NAME fixed_path_start_speed_guard_test COMMAND fixed_path_start_speed_guard_test)
endif()

ament_package()
