cmake_minimum_required(VERSION 3.0.2)
project(sa_klqr_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  actionlib
  control_msgs
  ur_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/sa_klqr_node.py
  scripts/koopman_loader.py
  scripts/state_builder.py
  scripts/lqr_gain.py
  scripts/centroid_fuzzy.py
  scripts/cartesian_wrapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
