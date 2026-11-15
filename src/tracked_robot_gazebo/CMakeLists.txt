cmake_minimum_required(VERSION 3.8)
project(tracked_robot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/gas_simulator.py
  scripts/generate_worlds.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
