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
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
// Transform broadcaster library
#include <tf/transform_broadcaster.h>

// Namespaces

// Default values
#define LOC_TOPIC_DEFAULT "/ttdrone_localization"
#define OUT_POSE_TOPIC_DEFAULT "/mavros/vision_pose/pose"
#define IMU_TOPIC_DEFAULT "/mavros/imu/data"
#define FRAME_RATE_DEFAULT 30
#define APPLY_ROT_OFFSET_DEFAULT false
#define APPLY_INITIAL_OFFSET_DEFAULT true

// Declaration of ROS Publishers
ros::Publisher loc_geom_pub;
ros::Publisher pos_angle_pub;

/**
      * @brief Input IMU callback
      * @param imu_msg IMU measurements message (sensor_msgs::Imu)
*/
geometry_msgs::Quaternion q_imu; // imu quaternion
bool first_time = true;          //first time the angle information is received
double roll_0, pitch_0, yaw_0;
void imuCB(const sensor_msgs::Imu::ConstPtr imu_msg)
{
    q_imu = imu_msg->orientation;

    if (first_time)
    {
        tf::Quaternion q_0(q_imu.x, q_imu.y, q_imu.z, q_imu.w);
        tf::Matrix3x3(q_0).getRPY(roll_0, pitch_0, yaw_0);
    }
}

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
    //posegps.pose.orientation = msg.pose.pose.orientation;
}

/**
      * @brief timer callback function
      * @param ros::TimerEvent
*/
ros::Timer timer;                //ROS timer object
std::vector<float> euler_offset; //angle offset
bool apply_rot_offset;           //flag to apply angle offsets or not
bool apply_initial_offset;       //flag to apply the initial orientation as offset
void timerCallback(const ros::TimerEvent &)
{
    // Get roll, pitch and yaw angles from imu measurements
    tf::Quaternion q_pos(q_imu.x, q_imu.y, q_imu.z, q_imu.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q_pos).getRPY(roll, pitch, yaw);

    // Fill in the orientation of the output localization
    double roll_off, pitch_off, yaw_off;
    if (apply_rot_offset)
    {
        if (apply_initial_offset)
        {
            // Apply substaction of offsets
            roll_off = roll - roll_0;
            pitch_off = pitch - pitch_0;
            yaw_off = yaw - yaw_0;
        }
        else
        {
            // Apply substaction of offsets
            roll_off = roll - euler_offset[0];
            pitch_off = pitch - euler_offset[1];
            yaw_off = yaw - euler_offset[2];
        }

        tf::Quaternion q = tf::createQuaternionFromRPY(roll_off, pitch_off, yaw_off);
        geometry_msgs::Quaternion q_loc;
        q_loc.x = q.x();
        q_loc.y = q.y();
        q_loc.z = q.z();
        q_loc.w = q.w();

        // Orientation is the imu orientation - angle offsets
        out_pose.pose.orientation = q_loc;
    }
    else
    {
        roll_off = roll;
        pitch_off = pitch;
        yaw_off = yaw;

        // Orientation is the imu orientation
        out_pose.pose.orientation = q_imu;
    }

    // Fill in the euler angles output message
    geometry_msgs::PoseStamped position_angle;
    position_angle.header = out_pose.header;
    position_angle.pose.position.x = roll_off * 180 / M_PI;
    position_angle.pose.position.y = pitch_off * 180 / M_PI;
    position_angle.pose.position.z = yaw_off * 180 / M_PI;

    // Update the timestamp of the message to be published
    out_pose.header.stamp = ros::Time::now();

    // Publish the localization orientation
    loc_geom_pub.publish(out_pose);
    pos_angle_pub.publish(position_angle);
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
    ros::init(argc, argv, "autonomous_flight_loc");
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
    std::string imu_topic;      // imu topic
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

    if (!node.getParam("imu_topic", imu_topic))
    {
        imu_topic = IMU_TOPIC_DEFAULT;
        ROS_INFO("[autonomous_flight_loc]: Param imu_topic set to DEAFULT");
    }

    if (!node.getParam("frame_rate", frame_rate))
    {
        frame_rate = FRAME_RATE_DEFAULT;
        ROS_INFO("[autonomous_flight_loc]: Param frame_rate set to DEAFULT");
    }

    if (!node.getParam("apply_rot_offset", apply_rot_offset))
    {
        apply_rot_offset = APPLY_ROT_OFFSET_DEFAULT;
        ROS_INFO("[autonomous_flight_loc]: Param apply_rot_offset set to DEAFULT");
    }

    if (!node.getParam("apply_initial_offset", apply_initial_offset))
    {
        apply_initial_offset = APPLY_INITIAL_OFFSET_DEFAULT;
        ROS_INFO("[autonomous_flight_loc]: Param apply_initial_offset set to DEAFULT");
    }

    euler_offset[3];
    if (!node.getParam("euler_offset", euler_offset))
    {
        euler_offset.assign(3, 0.0);
        ROS_INFO("[autonomous_flight_loc]: Param euler_offset set to DEAFULT");
    }

    // Print parameters
    ROS_INFO("\n Parameters: \n"
             "      localization_topic = %s \n"
             "      out_pose_topic = %s \n"
             "      imu_topic = %s \n"
             "      frame_rate = %f \n"
             "      euler_offset = [%f, %f, %f] \n"
             "      apply_rot_offset = %i \n"
             "      apply_initial_offset = %i \n",
             loc_topic.c_str(), out_pose_topic.c_str(),
             imu_topic.c_str(), frame_rate,
             euler_offset[0], euler_offset[1], euler_offset[2], apply_rot_offset, apply_initial_offset);

    // Definition of ROS publishers
    loc_geom_pub = node.advertise<geometry_msgs::PoseStamped>(out_pose_topic, 1);       // pose publisher
    pos_angle_pub = node.advertise<geometry_msgs::PoseStamped>("/uav_euler_angles", 1); //angle in degrees publisher

    // Definition of ROS subscribers
    ros::Subscriber sub = node.subscribe(loc_topic.c_str(), 2, locCB);     // input lozalization subscriber
    ros::Subscriber imu_sub = node.subscribe(imu_topic.c_str(), 2, imuCB); // input lozalization subscriber

    // Definition of the ROS timer
    timer = node.createTimer(ros::Duration(ros::Rate(frame_rate)), timerCallback, false, false);
    timer.start();

    // ROS spin loop
    ros::spin();

    return 0;
}
