cmake_minimum_required(VERSION 3.0.2)
project(swab_orchestrator_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/swab_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
