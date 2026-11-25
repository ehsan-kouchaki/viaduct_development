cmake_minimum_required(VERSION 3.0.2)
project(mission_manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

## Generate services in the 'srv' folder
add_service_files(
  FILES 
  mission_manager.srv
  Alarms.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  tf
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission_manager src/mission_manager.cpp)
add_dependencies(mission_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_manager ${catkin_LIBRARIES})