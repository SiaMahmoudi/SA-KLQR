cmake_minimum_required(VERSION 3.0.2)
project(realsense_plane_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  message_filters
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/plane_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
