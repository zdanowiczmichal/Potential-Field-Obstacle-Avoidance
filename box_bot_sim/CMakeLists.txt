cmake_minimum_required(VERSION 3.8)
project(box_bot_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf launch config worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/robot_move.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()