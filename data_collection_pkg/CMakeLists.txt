cmake_minimum_required(VERSION 3.0.2)
project(data_collection_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/data_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
