cmake_minimum_required(VERSION 3.0.2)
project(aruco_detector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  src/aruco_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)