cmake_minimum_required(VERSION 3.5)
project(byd_custom_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ooii: RobotMoveState 为避障用 12 态移动状态（2026-11 起替代 Int32 /car_op_status）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryState.msg"
  "msg/ChassisControl.msg"
  "msg/ControlRes.msg"
  "msg/MotionState.msg"
  "msg/NaviSegment.msg"
  "msg/QrCode.msg"
  "msg/RobotMoveState.msg"
  "action/Charging.action"
  "action/FollowFixedPath.action"
  "action/NavigationService.action"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
