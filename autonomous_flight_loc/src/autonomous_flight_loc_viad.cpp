/**
*   @file       autonomous_flight_loc.cpp
*
*   @author     Maria Polvillo Diaz <mapodi.94@gmail.com>
*   @memberof   GRVC
*
*   @date       13/05/2022
*   @brief      Main node of autonomous flight localization
*/

// Declaration of Libraries
// General libraries
#include <ros/ros.h>
#include <math.h>
#include <vector>
// ROS libraries
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
// Transform broadcaster library
#include <tf/transform_broadcaster.h>

// Namespaces

// Default values
#define LOC_TOPIC_DEFAULT "/ttdrone_localization"
#define OUT_POSE_TOPIC_DEFAULT "/mavros/vision_pose/pose"
#define FRAME_RATE_DEFAULT 100

// Declaration of ROS Publishers
ros::Publisher loc_geom_pub;
ros::Publisher pos_angle_pub;

/**
      * @brief Input localization callback
      * @param msg localization message (nav_msgs::Odometry)
*/
geometry_msgs::PoseStamped out_pose;
void locCB(const nav_msgs::Odometry msg)
{
    /** Geometry_msgs Posestamped message **/
    out_pose.header = msg.header;
    out_pose.pose.position = msg.pose.pose.position;
    out_pose.pose.orientation = msg.pose.pose.orientation;
}

/**
      * @brief timer callback function
      * @param ros::TimerEvent
*/
ros::Timer timer;                //ROS timer object
bool apply_rot_offset;           //flag to apply angle offsets or not
bool apply_initial_offset;       //flag to apply the initial orientation as offset
void timerCallback(const ros::TimerEvent &)
{
    // Update the timestamp of the message to be published
    out_pose.header.stamp = ros::Time::now();

    // Publish the localization orientation
    loc_geom_pub.publish(out_pose);
}

/**
      * @brief Main function
      * @param argc argument count 
      * @param argv character array that holds all the arguments
      * @return exit of program is int variable
*/
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "autonomous_flight_loc_viad");
    ros::NodeHandle node("~");
    std::string node_name = ros::this_node::getName();
    ROS_INFO("[%s]: node initialized", node_name.c_str());

    // Check is roscore is running
    if (!ros::master::check())
    {
        ROS_ERROR("[%s]: roscore is not running.", node_name.c_str());
        return 0;
    }

    // XXX - Create objet of type XXX and launch ROS spin loop

    // Retrieve parameters
    std::string loc_topic;      // input localization topic
    std::string out_pose_topic; // output localization topic
    float frame_rate;           // frame rate of publication

    if (!node.getParam("localization_topic", loc_topic))
    {
        loc_topic = LOC_TOPIC_DEFAULT;
        ROS_INFO("[autonomous_flight_loc]: Param localization_topic set to DEAFULT");
    }

    if (!node.getParam("out_pose_topic", out_pose_topic))
    {
        out_pose_topic = OUT_POSE_TOPIC_DEFAULT;
        ROS_INFO("[autonomous_flight_loc]: Param out_pose_topic set to DEAFULT");
    }

    if (!node.getParam("frame_rate", frame_rate))
    {
        frame_rate = FRAME_RATE_DEFAULT;
        ROS_INFO("[autonomous_flight_loc]: Param frame_rate set to DEAFULT");
    }
 
    frame_rate = 100;

    // Print parameters
    ROS_INFO("\n Parameters: \n"
             "      localization_topic = %s \n"
             "      out_pose_topic = %s \n"
             "      frame_rate = %f \n",
             loc_topic.c_str(), out_pose_topic.c_str(), frame_rate);

    // Definition of ROS publishers
    loc_geom_pub = node.advertise<geometry_msgs::PoseStamped>(out_pose_topic, 1);       // pose publisher
    
    // Definition of ROS subscribers
    ros::Subscriber sub = node.subscribe(loc_topic.c_str(), 2, locCB);     // input lozalization subscriber
    
    // Definition of the ROS timer
    timer = node.createTimer(ros::Duration(ros::Rate(frame_rate)), timerCallback, false, false);
    timer.start();

    // ROS spin loop
    ros::spin();

    return 0;
}
