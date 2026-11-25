cmake_minimum_required(VERSION 3.0.2)
project(deploy_auto)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  tf
)

catkin_package(CATKIN_DEPENDS rospy geometry_msgs std_msgs tf)

catkin_install_python(PROGRAMS
  scripts/automatic_alignment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
