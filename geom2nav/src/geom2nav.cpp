/**
*   @file       geom2nav.cpp
*
*   @author     Maria Polvillo Diaz <mapodi.94@gmail.com>
*   @memberof   GRVC
*
*   @date       04/04/2022
*   @brief      The objective of this node is to change the message format
*               from geometry_msgs/PoseStamped to nav_msgs/Odometry
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>

// Default parameters
#define GEOM_TOPIC_DEFAULT "/leica/position"
#define NAV_TOPIC_DEFAULT "/leica/position/odom"
#define FRAME_ID_DEFAULT "/map"

// Geometry-Msgs callback
ros::Publisher pub;
std::string frame_id;

/**
      * @brief Localization callback function
      * @param msg geometry_msgs/PoseStamped message containing the localization measurements 
*/
void geom2navCb(const geometry_msgs::PoseStamped msg)
{
    nav_msgs::Odometry nav_msgs;
    nav_msgs.pose.pose.position = msg.pose.position;
    nav_msgs.header = msg.header;
    nav_msgs.header.frame_id = frame_id;

    pub.publish(nav_msgs);
}

/**
      * @brief Main function
      * @param argc argument count 
      * @param argv character array that holds all the arguments
      * @return exit of program is int variable
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "geom2nav");
    ros::NodeHandle nh("~");

    // Getting parameters
    std::string geom_topic, nav_topic;
    nh.param<std::string>("geom_topic", geom_topic, GEOM_TOPIC_DEFAULT);
    nh.param<std::string>("nav_topic", nav_topic, NAV_TOPIC_DEFAULT);
    nh.param<std::string>("frame_id", frame_id, FRAME_ID_DEFAULT);

    ros::Subscriber sub = nh.subscribe(geom_topic, 1, geom2navCb); // subscriber
    pub = nh.advertise<nav_msgs::Odometry>(nav_topic, 1);          // publisher

    ros::spin();
}
