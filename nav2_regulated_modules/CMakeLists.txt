# 中文注释：本文件定义独立规控包的编译依赖、可执行文件源集和安装布局。
cmake_minimum_required(VERSION 3.8)
project(nav2_regulated_modules)

# 中文注释：GNU／Clang 编译器启用常用告警，尽早暴露潜在的类型和接口问题。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 中文注释：这些依赖分别提供 ROS 2 节点、Action、Lifecycle、Nav2 接口、消息和 TF 变换能力。
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(spdlog_wrapper REQUIRED)
find_package(visualization_msgs REQUIRED)

# 中文注释：生成固定路径业务 Action；外层接口只暴露完整 Path、剩余距离和成功标志。
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowFixedPath.action"
  DEPENDENCIES nav_msgs
)

# 中文注释：独立规控节点只编排现有 Nav2 服务器，不链接行为树运行库。
add_executable(regulated_navigator_node
  src/regulated_navigator_node.cpp
  src/regulated_navigator.cpp
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
# 中文注释：节点采用 C++17；C99 声明由 ament 生成的公共目标属性保持兼容。
target_compile_features(regulated_navigator_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(regulated_navigator_node
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  nav2_msgs
  nav2_util
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  spdlog_wrapper
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(regulated_navigator_node "${cpp_typesupport_target}")

# 中文注释：将 Collision Monitor 的三个 PolygonStamped 转为可调线宽并带文字的 RViz MarkerArray。
add_executable(collision_boundary_visualizer_node src/collision_boundary_visualizer.cpp)
target_compile_features(collision_boundary_visualizer_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(collision_boundary_visualizer_node rclcpp geometry_msgs visualization_msgs)

# 中文注释：安装可执行文件供 ros2 run 调用，并安装头文件及运行所需配置资源。
install(TARGETS regulated_navigator_node collision_boundary_visualizer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include
  DESTINATION include
)

install(DIRECTORY
  action
  launch
  maps
  params
  rviz
  src
  DESTINATION share/${PROJECT_NAME}
)

# 中文注释：导出 ament 包元数据，使 colcon 和下游包能够发现本包。
ament_package()
