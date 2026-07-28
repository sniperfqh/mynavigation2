# 中文注释：声明最低 CMake 版本并创建键盘遥控 ROS 2 包工程。
cmake_minimum_required(VERSION 3.8)
project(myagv_keyboard_control)

# 中文注释：GNU 和 Clang 编译器启用常用告警，尽早发现潜在类型与接口问题。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 中文注释：查找 ament 构建系统、底盘自定义消息和 ROS 2 C++ 客户端库。
find_package(ament_cmake REQUIRED)
find_package(byd_custom_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# 中文注释：把键盘读取、速度平滑和底盘指令发布源码编译为独立节点。
add_executable(myagv_keyboard_control_node src/myagv_keyboard_control_node.cpp)
# 中文注释：为节点链接 ControlRes 消息接口与 rclcpp 运行时依赖。
ament_target_dependencies(
  myagv_keyboard_control_node
  byd_custom_msgs
  rclcpp
)

# 中文注释：节点使用 std::clamp、智能指针等 C++17 能力。
target_compile_features(myagv_keyboard_control_node PUBLIC cxx_std_17)

# 中文注释：安装节点可执行文件，供 ros2 run 和 Launch 按包名查找。
install(
  TARGETS myagv_keyboard_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 中文注释：安装参数和 Launch 目录，保证 install 空间具备完整启动资源。
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 中文注释：安装包内使用说明，便于在 install 空间查询接口和调参方法。
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

# 中文注释：生成 ament 包索引与导出信息。
ament_package()
