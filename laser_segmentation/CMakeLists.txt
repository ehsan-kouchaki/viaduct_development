cmake_minimum_required(VERSION 3.10)
project(laser_segmentation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  laser_geometry
  pcl_ros
  pcl_conversions
  geometry_msgs
  std_msgs
  message_generation
  tf
  mission_manager
)

# add_service_files(
#   FILES 
#   Alarms.srv
# )

# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES 
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include/laser_segmentation
  ${catkin_INCLUDE_DIRS}
)  

add_executable(2d_icp src/2D_ICP.cpp)
target_link_libraries(2d_icp ${catkin_LIBRARIES})

add_executable(2d_icp_iter src/2D_ICP_iterative.cpp)
target_link_libraries(2d_icp_iter ${catkin_LIBRARIES})

add_executable(map_generation src/map_generation.cpp)
target_link_libraries(map_generation ${catkin_LIBRARIES})

add_executable(ground_truth src/ground_truth.cpp)
target_link_libraries(ground_truth ${catkin_LIBRARIES})

add_executable(3dPointCloud src/3dPointCloud.cpp)
target_link_libraries(3dPointCloud ${catkin_LIBRARIES})

add_executable(accumulate src/accumulate.cpp)
target_link_libraries(accumulate ${catkin_LIBRARIES} )

add_executable(laser_generic src/laser_generic.cpp src/dbscan.cpp src/LaserGeneric.cpp)
target_link_libraries(laser_generic ${catkin_LIBRARIES})

