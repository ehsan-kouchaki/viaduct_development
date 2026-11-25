/**
*   @file       paths_publisher.cpp
*
*   @author     Maria Polvillo Diaz <mapodi.94@gmail.com>
*   @memberof   GRVC
*
*   @date       23/12/2021
*   @brief      The objective of this node is to receive two localization informations
*               and publish them as paths to visualize them.
*/

#include <ros/ros.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#define LOC_TOPIC_DEFAULT "/uav_localization"
#define GT_TOPIC_DEFAULT "/leica/pose/odom"
#define LOC_PATH_DEFAULT "/uav_localization_path"
#define GT_PATH_DEFAULT "/leica/pose/path"
#define FRAME_ID_DEFAULT "/world"

ros::Publisher pub_loc;
ros::Publisher pub_gt;
nav_msgs::Path path_out;
nav_msgs::Path path_gt_out;
std::string frame_id;

/**
      * @brief Localization information handler
      * @param loc_in localization information in nav_ms/Odometry format
*/
geometry_msgs::PoseStamped position_aux_gt;
void gt_cb(const nav_msgs::Odometry &loc_in)
{

  // LOCALIZATION ALGORITHM INFORMATION
  if (!((std::isnan(loc_in.pose.pose.position.x)) ||
        (std::isnan(loc_in.pose.pose.position.y)) ||
        (std::isnan(loc_in.pose.pose.orientation.x)) ||
        (std::isnan(loc_in.pose.pose.orientation.y)) ||
        (std::isnan(loc_in.pose.pose.orientation.z)) ||
        (std::isnan(loc_in.pose.pose.orientation.w))))
  {

    position_aux_gt.header.stamp = ros::Time::now();
    position_aux_gt.header.frame_id = frame_id; //loc_in.header.frame_id;
    position_aux_gt.pose.position.x = loc_in.pose.pose.position.x;
    position_aux_gt.pose.position.y = loc_in.pose.pose.position.y;
    position_aux_gt.pose.position.z = loc_in.pose.pose.position.z;
    position_aux_gt.pose.orientation.x = loc_in.pose.pose.orientation.x;
    position_aux_gt.pose.orientation.y = loc_in.pose.pose.orientation.y;
    position_aux_gt.pose.orientation.z = loc_in.pose.pose.orientation.z;
    position_aux_gt.pose.orientation.w = loc_in.pose.pose.orientation.w;

    //path_out.header.stamp = ros::Time::now();
    //path_out.header.frame_id = "/map";
    //path_out.poses.push_back(position_aux_gt);
  }

  // *************** PUBLISH PATHS *****************
  //pub_gt.publish(path_out);
}

/**
      * @brief Localization handler
      * @param loc_in localization information in nav_ms/Odometry format
*/
void loc_cb(const nav_msgs::Odometry &loc_in)
{

  geometry_msgs::PoseStamped position_aux;

  // LOCALIZATION ALGORITHM INFORMATION
  if (!((std::isnan(loc_in.pose.pose.position.x)) ||
        (std::isnan(loc_in.pose.pose.position.y)) ||
        (std::isnan(loc_in.pose.pose.orientation.x)) ||
        (std::isnan(loc_in.pose.pose.orientation.y)) ||
        (std::isnan(loc_in.pose.pose.orientation.z)) ||
        (std::isnan(loc_in.pose.pose.orientation.w))))
  {

    position_aux.header.stamp = ros::Time::now();
    position_aux.header.frame_id = frame_id; //loc_in.header.frame_id;
    position_aux.pose.position.x = loc_in.pose.pose.position.x;
    position_aux.pose.position.y = loc_in.pose.pose.position.y;
    position_aux.pose.position.z = loc_in.pose.pose.position.z;
    position_aux.pose.orientation.x = loc_in.pose.pose.orientation.x;
    position_aux.pose.orientation.y = loc_in.pose.pose.orientation.y;
    position_aux.pose.orientation.z = loc_in.pose.pose.orientation.z;
    position_aux.pose.orientation.w = loc_in.pose.pose.orientation.w;

    path_out.header.stamp = ros::Time::now();
    path_out.header.frame_id = frame_id;
    path_out.poses.push_back(position_aux);

    path_gt_out.header.stamp = ros::Time::now();
    path_gt_out.header.frame_id = frame_id;
    //position_aux_gt.pose.position.z = position_aux.pose.position.z;
    path_gt_out.poses.push_back(position_aux_gt);
  }

  // *************** PUBLISH PATHS *****************
  pub_loc.publish(path_out);
  pub_gt.publish(path_gt_out);
}

/**
      * @brief Main function
      * @param argc argument count 
      * @param argv character array that holds all the arguments
      * @return exit of program as int variable
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "paths_publisher");
  ros::NodeHandle nh("~");

  std::string loc_topic, loc_path;
  std::string gt_topic, gt_path;

  //-- PARAMETERS --
  if (!nh.getParam("localization_topic", loc_topic))
  {
    loc_topic = LOC_TOPIC_DEFAULT;
    ROS_INFO("[path_publisher]: Param localization_topic set to DEAFULT");
  }

  if (!nh.getParam("gt_topic", gt_topic))
  {
    gt_topic = GT_TOPIC_DEFAULT;
    ROS_INFO("[path_publisher]: Param gt_topic set to DEAFULT");
  }

  if (!nh.getParam("localization_path", loc_path))
  {
    loc_path = LOC_PATH_DEFAULT;
    ROS_INFO("[path_publisher]: Param localization_path set to DEAFULT");
  }

  if (!nh.getParam("gt_path", gt_path))
  {
    loc_path = GT_PATH_DEFAULT;
    ROS_INFO("[path_publisher]: Param gt_path set to DEAFULT");
  }

  if (!nh.getParam("frame_id", frame_id))
  {
    frame_id = FRAME_ID_DEFAULT;
    ROS_INFO("[path_publisher]: Param frame_id set to DEAFULT");
  }

  // Print parameters
  ROS_INFO("\n Parameters: \n"
           "      localization_topic = %s \n"
           "      gt_topic = %s \n"
           "      loc_path = %s \n"
           "      gt_path = %s \n"           
           "      frame_id = %s \n",
           loc_topic.c_str(), gt_topic.c_str(), loc_path.c_str(), 
           gt_path.c_str(), frame_id.c_str());

  ros::Subscriber subOdom = nh.subscribe(loc_topic, 1, loc_cb);
  ros::Subscriber subGTLoc = nh.subscribe(gt_topic, 1, gt_cb);

  pub_loc = nh.advertise<nav_msgs::Path>(loc_path, 1);
  pub_gt = nh.advertise<nav_msgs::Path>(gt_path, 1);

  ros::spin();
  return 0;
}
