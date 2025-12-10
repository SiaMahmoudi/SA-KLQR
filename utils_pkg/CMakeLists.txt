cmake_minimum_required(VERSION 3.0.2)
project(utils_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/math_utils.py
  scripts/filters.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
