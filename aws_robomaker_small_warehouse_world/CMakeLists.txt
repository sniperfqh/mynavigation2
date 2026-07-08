cmake_minimum_required(VERSION 3.5)
project(aws_robomaker_small_warehouse_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models worlds maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
