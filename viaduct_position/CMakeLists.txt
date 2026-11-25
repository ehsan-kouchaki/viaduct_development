cmake_minimum_required(VERSION 3.10)
project(viaduct_position)

find_package(OpenCV REQUIRED)

set(OpenCV_LIBS
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
  opencv_highgui
  opencv_features2d
  opencv_calib3d
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  laser_geometry
  pcl_ros
  pcl_conversions
  cv_bridge
  geometry_msgs
  image_transport
  rospy
  std_msgs
  tf
  mission_manager
)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include/viaduct_position
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)  

add_executable(direct_hough src/direct_hough_360.cpp)
target_link_libraries(direct_hough ${catkin_LIBRARIES})

add_executable(ceil_hough src/ceil_hough.cpp)
target_link_libraries(ceil_hough ${catkin_LIBRARIES})

add_executable(homo_piloting src/homo_piloting.cpp)
target_link_libraries(homo_piloting ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(track_piloting src/track_piloting.cpp)
target_link_libraries(track_piloting ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ground_truth src/ground_truth.cpp)
target_link_libraries(ground_truth ${catkin_LIBRARIES})

add_executable(3dPointCloud src/3dPointCloud.cpp)
target_link_libraries(3dPointCloud ${catkin_LIBRARIES})

add_executable(navegation src/navegation.cpp)
target_link_libraries(navegation ${catkin_LIBRARIES})

add_executable(IMU_orientation src/IMU_orientation.cpp)
target_link_libraries(IMU_orientation ${catkin_LIBRARIES})

add_executable(laser_generic src/laser_generic.cpp src/dbscan.cpp)
target_link_libraries(laser_generic ${catkin_LIBRARIES})