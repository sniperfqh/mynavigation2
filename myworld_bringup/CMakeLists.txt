cmake_minimum_required(VERSION 3.5)
project(myworld_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/chassis_control_to_twist.py
  scripts/cpu_lidar.py
  scripts/odom_to_motion_state.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
