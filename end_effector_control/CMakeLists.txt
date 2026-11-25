cmake_minimum_required(VERSION 3.0.2)
project(end_effector_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

catkin_package(
  CATKIN_DEPENDS roscpp serial std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(end_effector_node
    src/end_effector_node.cpp
    src/keyboard.cpp
)

target_link_libraries(end_effector_node
    ${catkin_LIBRARIES}
)

install(TARGETS end_effector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)