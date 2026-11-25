/**
*   @file       DirectHough.h
*
*   @author     Juan Antonio Sanchez Diaz <juanantoniodiaz99@gmail.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Header file of DirectHough class
*/

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include "laser_geometry/laser_geometry.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "mission_manager/mission_manager.h"
#include <tf/transform_broadcaster.h>

/* class DirectHough
{
    
private:
    // Declaration of ROS Node Handle 
    ros::NodeHandle node_; // ROS node handle

    // Proyector de puntos en el espacio
    laser_geometry::LaserProjection projector_; 

    // PUBLISHER
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub2d;
    ros::Publisher path;
    ros::Publisher high;
    ros::Publisher houghp;
    nav_msgs::Path msg_path;
    geometry_msgs::PoseStamped m;
    geometry_msgs::PoseStamped mz;

    // GLOBALS MSGS
    sensor_msgs::PointCloud data;
    sensor_msgs::PointCloud data_mirror;

    // LASER FILTER PARAMETERS
    int max = 271;
    int min = 220;
    int intensity = 9000;
    float distance = 9000;
    float aux = 0;
    float d = 0.2;
    int min_points = 10;
    std::string scan_topic = "scan";
    std::string odom_frame_id = "world";
    std::string path_frame_id = "laser";

    // POSITION
    float xa = 0, ya = 0, za = 0, thetaa = 0;
    float xaa = 0, yaa = 0, zaa = 0, thetaaa = 0;
    float xi = 0, yi = 0, zi = 0, thetai = 0;
    float XG = 0, YG = 0, ZG = 0, YAWG = 0;
    float error = 5;
    bool ini = true;
    bool takeoff = false;
    bool debug = true;

    float yaw = 0;

    float d_roofa = 0;
    int cont_med = 0;

    unsigned int* _accu;
    int ir_max_a = 3414, ir_max_a2 = 0;
    int it_max_a = 0;

    float XY_hough = 0, XZ_hough = 0; 

    // Subscriber callbacks 
    // \cb lidar_callback : laser 2D measurements callback
    void lidar_callback(const  sensor_msgs::LaserScan::ConstPtr& msg);

    // Headers of Private Functions 
    // \fnc getParameters : retrieve parameters to configure the filter
    void getParameters();

public:
    // Declaration of public variables 

    // Headers of Public Functions 
    // \fnc DirectHough : Constructor
    DirectHough(ros::NodeHandle &_node, tf::TransformBroadcaster *tfbr_);

    // \fnc ~DirectHough : Destructor
    ~DirectHough();

    //\fnc spin : ROS spin loop
    void spin();
}  */