/**
*   @file       ViaductKFLocalization.h
*
*   @author     Maria Polvillo Diaz <mapodi.94@gmail.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Header file of KFLocalization class
*/

#pragma once

// Declaration of Libraries
// General libraries
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <string>
// Eigen libraries
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
// ROS libraries
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
// Transform broadcaster library
#include <tf/transform_broadcaster.h>
// ROS Services
#include <mission_manager/mission_manager.h>

// Namespaces

// Default values
#define GPS_POS_TOPIC_DEFAULT "/gps_position"
#define LIDAR_POS_TOPIC_DEFAULT "/lidar_position"
#define HOMOG_POS_TOPIC_DEFAULT "/homog_position"
#define HOMOG_DELTA_TOPIC_DEFAULT "/homog_odom"
#define IMU_TOPIC_DEFAULT "/mavros/imu/data"
#define ALTIMETER_TOPIC_DEFAULT "/sf11"
#define INIYAW_TOPIC_DEFAULT "/imu/yaw"
#define CEILINGPOSE_TOPIC_DEFAULT "/hough/ceil"
#define EST_POSE_TOPIC_DEFAULT "/viaddrone_position"
#define FRAME_ID_DEFAULT "/base_link_init"
#define CHILD_FRAME_ID_DEFAULT "/base_link"
#define SIGMAMODEL_DEFAULT 0.3
#define SIGMAALTIMETER_DEFAULT 0.3
#define SIGMACEILPOS_DEFAULT 0.3
#define CONTRIBMODULE_DEFAULT 0.4
#define PUBLISHTF_DEFAULT true
#define TIMETHRESHOLD_DEFAULT 0.75
#define FILTEROUTLIERS_DEFAULT false
#define DELTAOUTLIER_DEFAULT 0.5

/**
      * @brief Declaraion of KFLocalization class
*/
class KFLocalization
{

private:
    /* Declaration of ROS Node Handle */
    ros::NodeHandle node_; // ROS node handle

    /* Declaration of objects */
    // \object XXX : XXX

    /* List of params. Fully documented in yaml configuration file */
    // \param iniposition_topic_ : name of the topic that provides the initial position
    std::vector<double> iniposition_;
    // \param gpspos_topic_ : name of input topic with GPS localization
    std::string gpspos_topic_;
    // \param lidarpos_topic_ : name of input topic with LIDAR-based localization
    std::string lidarpos_topic_;
    // \param homogpos_topic_ : name of input topic with homography-based localization
    std::string homogpos_topic_;
    // \param homogdelta_topic_ : name of input topic with increase of position (from homography information)
    std::string homogdelta_topic_;
    // \param imu_topic_ : name of input topic with IMU sensor measurements
    std::string imu_topic_;
    // \param altimeter_topic_ : name of input topic with altimeter sensor measurements
    std::string altimeter_topic_;
    // \param iniyaw_topic_ : name of the input topic with the initial yaw of the UAV
    std::string iniyaw_topic_;
    // \param ceiling_pose_topic_ : name of the topic of the estimated position using the ceiling of the bridge
    std::string ceiling_pose_topic_;
    // \param est_pose_topic_ : name of the topic of the estimated position
    std::string est_pose_topic_;
    // \param frame_id_ : frame of reference of the estimated position
    std::string frame_id_;
    // \param child_frame_id_ : child frame of reference of the estimated position
    std::string child_frame_id_;
    // \param sigma_model_ : model tipical deviation
    double sigma_model_;
    // \param sigma_gps_ : GPS sensor tipical deviation
    std::vector<double> sigma_gps_;
    // \param sigma_lidar_ : LIDAR sensor tipical deviation
    std::vector<double> sigma_lidar_;
    // \param sigma_homog_ : homography sensor tipical deviation
    std::vector<double> sigma_homog_;
    // \param sigma_altim_ : altimeter sensor tipical deviation
    double sigma_altim_;
    // \param sigma_ceilpos_ :  tipical deviation of the pose estimated with the ceiling of the bridge
    double sigma_ceilpos_;
    // \param contrib_module_ : maximum contribution to update the filter
    double contrib_module_;
    // \param publish_tf_ : flag to publish tf or not
    bool publish_tf_;
    // \param time_threshold_ : if time between measurements is greater than a threshold, then
    // the control matriz is null
    double time_threshold_;
    // \param filter_outliers_ : flag to select if the outlier filter should work
    bool filter_outliers_;
    // \param deltam_outlier_ : threshold to the outlier filter
    double deltam_outlier_;

    // \param time_threshold_ : if time between measurements is greater than a threshold, then
    // the control matriz is null
    //double time_threshold_;

    /* Declaration of private variables */
    //double contribution_; // contribution of the update stage

    Eigen::VectorXd mut_;  // current state vector
    Eigen::VectorXd mut1_; // state vector in t-1
    Eigen::VectorXd pmut_; // curret predicted state vector

    Eigen::MatrixXd sigmat_;  // current variance matrix
    Eigen::MatrixXd sigmat1_; // sigma in t-1
    Eigen::MatrixXd psigmat_; // predicted variance matrix

    Eigen::VectorXd B_;        // control matrix
    Eigen::MatrixXd A_;        // jacobian of the model // G_
    Eigen::MatrixXd I_;        // identity matrix
    Eigen::MatrixXd R_;        // variance matrix of the prediction model
    Eigen::MatrixXd Qtakeoff_; // variance matrix of the observation model during the take off
    Eigen::MatrixXd Qnav_;     // variance matrix of the observation model during the take off
    Eigen::MatrixXd Ct_;       // TODO // Ct_
    Eigen::MatrixXd kt_;       // gain matrix of the update stage
    Eigen::VectorXd zt_;       // measurement vector

    bool first_time_;        // flag to detect if ekf filter has been initialized
    bool first_meas_;        // flag to detect the first measurement and get a local position estimation
    bool takeoff_stage_;     // flag to detect if the UAV is taking off (true) or is navigating (false)
    double x_0_, y_0_, z_0_; // initial offset to get a local position estimation
    bool lidar_received_;    // if lidar measurement has been received

    nav_msgs::Odometry gps_pos_;        // GPS pose with time stamp
    nav_msgs::Odometry lidar_pos_;      // LIDAR pose with time stamp
    nav_msgs::Odometry homog_pos_;      // LIDAR pose with time stamp
    geometry_msgs::Point deltapos_homog_;  // Increment of position from homog method
    nav_msgs::Odometry ceil_pos_;       // Initial pose estimated with the ceiling of the bridge
    geometry_msgs::Vector3 linear_acc_; // linear acceleration information from IMU sensor

    double t1_;               // time stamp in t-1
    double t_;                // current time stamp
    std_msgs::Header curr_t_; // variable to store time stamp information

    double roll_imu_, pitch_imu_, yaw_imu_; // roll and pitch angles from the IMU sensor
    double yaw_homog_;                      // yaw estimated with the homography method
    double deltayaw_homog_;                  // increment in yaw estimation from homography method
    double altim_height_;                   // height measurement from altimeter
    double yaw_lidar_;                      // yaw estimated with hough method
    double yaw_ini_;                        // yaw estimated with the IMU measurements from autopilot
    double ceil_distance_;                  // ceil distance measured with mirror

    /* tf::Transform transform;
    tf::Quaternion q_rot; */

    /* Declaration of TF Broadcasters */
    //tf::TransformBroadcaster *_br;
    tf::TransformBroadcaster *br_;

    /* Declaration of ROS Subscribers */
    // \subscr gpspos_sub_ : subscriber to gps localization
    ros::Subscriber gpspos_sub_;
    // \subscr gpspos_sub_ : subscriber to ceil distance
    ros::Subscriber ceildis_sub_;
    // \subscr lidarpos_sub_ : subscriber to lidar localization
    ros::Subscriber lidarpos_sub_;
    // \subscr homogpos_sub_ : subscriber to homography localization
    ros::Subscriber homogpos_sub_;
    // \subscr homogpos_sub_ : subscriber to increment of position from homography method
    ros::Subscriber homogdelta_sub_;
    // \subscr imu_sub_ : subscriber to IMU measurements
    ros::Subscriber imu_sub_;
    // \subscr altim_sub_ : subscriber to altimeter measurements
    ros::Subscriber altim_sub_;
    // \subscr iniyaw_sub_ : subscriber to initial yaw of the UAV
    ros::Subscriber iniyaw_sub_;
    // \subscr ceilingpos_sub_ : subscriber to the estimated position with the ceiling of the bridge
    ros::Subscriber ceilingpos_sub_;

    /* Declaration of ROS Publishers */
    // \pub state_pub_ : estimated state publisher
    ros::Publisher state_pub_;

    /* Declaration of ROS services */
    // \srv XXsrv_ : XXdoc
    ros::ServiceServer service_;

    /* Subscriber callbacks */
    // \cb ekfCB : beacon measurements callback
    void kfCB(const sensor_msgs::Range::ConstPtr &_msg);

    /* Service callbacks */
    // \cb node_init : service callback
    bool node_init(mission_manager::mission_manager::Request &req,
                   mission_manager::mission_manager::Response &res);

    /* Private Functions */
    // \fnc getParameters : retrieve parameters to configure the filter
    void getParameters();

public:
    /* Declaration of public variables */

    /* Public Functions */
    // \fnc KFLocalization : Constructor
    KFLocalization(ros::NodeHandle &_node, tf::TransformBroadcaster *tfbr_);
    // \fnc ~KFLocalization : Destructor
    ~KFLocalization();
    //\fnc spin : ROS spin loop
    void spin();
};
