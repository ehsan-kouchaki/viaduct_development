#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <iomanip>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "mission_manager/mission_manager.h"
#include "geometry_msgs/Quaternion.h"

ros::Subscriber sub;
float z = 0;
float z_max = 0;

ros::ServiceClient client;
ros::ServiceClient client_camera;
ros::ServiceClient client_laser;

std::string pose_topic = "/pose";
bool init = true;

void kf_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(msg->pose.pose.position.z >= z_max && init)
    {
        mission_manager::mission_manager srv;

        float xrot = 0;
        float yrot = 0;
        float zrot = 0; 

        xrot = msg->pose.pose.position.x;
        yrot = msg->pose.pose.position.y;

        // XY Rotation
        float xtrans = xrot;
        float ytrans = yrot;
        float theta = 90 * 3.1416/180; 
        xrot = xtrans*cos(theta) + ytrans*sin(theta);
        yrot = -xtrans*sin(theta) + ytrans*cos(theta); 

        srv.request.x_pos = xrot;
        srv.request.y_pos = yrot;
        srv.request.z_pos = msg->pose.pose.position.z;

        double roll_imu, pitch_imu_, yaw_imu_;
        geometry_msgs::Quaternion _q_imu = msg->pose.pose.orientation;
        tf::Quaternion q_homog(_q_imu.x, _q_imu.y, _q_imu.z, _q_imu.w);                                                     
        tf::Matrix3x3(q_homog).getRPY(roll_imu, pitch_imu_, yaw_imu_);

        srv.request.yaw_rot = yaw_imu_;
        srv.request.takeoff = false;

        client.call(srv);
        client_camera.call(srv);
        client_laser.call(srv);
        init = false;
    }
}

int main(int argc, char **argv)
{
    std::string _node_name = "mission_manager_localization" ;
    ros::init(argc, argv, _node_name);
    ros::NodeHandle n;

    n.getParam(_node_name + "/POSITION_TOPIC", pose_topic);
    n.getParam(_node_name + "/HIGH_CHANGE", z_max);

    sub = n.subscribe(pose_topic, 1, kf_callback);

    client_camera = n.serviceClient<mission_manager::mission_manager>("mission_manager_camera");
    client_laser = n.serviceClient<mission_manager::mission_manager>("mission_manager_laser");
    client = n.serviceClient<mission_manager::mission_manager>("mission_manager");

    ros::spin();

    return 0;
}
