/**
*   @file       laser.h
*
*   @author     Juan Antonio Sanchez Diaz <jsanchez22@us.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Header file of LaserGeneric class
*/

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
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
#include "dbscan.h"

class LaserGeneric
{
private:
    // Declaration of ROS Node Handle 
    ros::NodeHandle node_; // ROS node handle

   // POLAR TO CARTESIAN COORDINATES
    laser_geometry::LaserProjection projector_;

    // PUBLISHER
    ros::Publisher pub;
    ros::Publisher pub2;
    ros::Publisher pub2d;
    ros::Publisher clus;
    ros::Publisher path;
    ros::Publisher high;
    ros::Publisher posepub;

    // LOCALIZATION
    nav_msgs::Path msg_path;
    geometry_msgs::PoseStamped m;
    geometry_msgs::PoseStamped mz;

    // TOPIC'S NAME
    std::string scan_topic = "/scan";
    std::string filtered_laser_topic = "/laser_filtered";
    std::string filtered_mirror_topic = "/laser_filtered_mirror";
    std::string path_topic = "/laser/path";
    std::string pose_topic = "/laser/pose";
    std::string cluster_topic = "/cluster";
    std::string z_topic = "/laser/z";
    std::string mission_manager_service = "mission_manager_laser";

    // FRAME ID
    std::string path_frame_id = "world";
    std::string laser_frame_id = "laser";

    // LASER FILTER PARAMETERS
    int i_max = 271;
    int i_min = 220;
    float max_dis = 9000;
    float aux = 0;
    float d = 0.2;
    int min_points = 10;
    int ofs = 100;

    // CLUSTERING
    float epsilon = 1;
    float minimum_points = 8;

    // POSITION
    float xa = 0, ya = 0, za = 0, thetaa = 0;
    float xaa = 0, yaa = 0, zaa = 0, thetaaa = 0;
    float xi = 0, yi = 0, zi = 0, thetai = 0;
    float XG = 0, YG = 0, ZG = 0, YAWG = 0;
    float error = 2000;

    float yaw = 0;

    float d_roofa = 0;
    int cont_med = 0;

    unsigned int* _accu;
    int ir_max_a = 0, ir_max_a2 = 0;
    int it_max_a = 0, it_max_a2 = 0;

    float XY_hough = 0, XZ_hough = 0; 

    // SETTINGS
    bool ini = true;
    bool takeoff = false;
    bool debug = true;

    // Subscriber callbacks 
    // \cb lidar_callback : laser 2D measurements callback
    void lidar_callback(const  sensor_msgs::LaserScan::ConstPtr& msg);

    // Headers of Private Functions 
    // \fnc getParameters : retrieve parameters to configure the filter
    void getParameters();
    std::vector<int> unicos(std::vector<Point> cloud);
    double angle_estimation(std::vector<Point> cloud, int cluster, vector<double> centroid);
    bool node_init(mission_manager::mission_manager::Request  &req, mission_manager::mission_manager::Response &res);

public:
    // Declaration of public variables 

    // Headers of Public Functions 
    // \fnc LaserGeneric : Constructor
    LaserGeneric(ros::NodeHandle &_node);

    // \fnc ~LaserGeneric : Destructor
    ~LaserGeneric();

    //\fnc spin : ROS spin loop
    void spin();
};