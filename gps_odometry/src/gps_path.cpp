// Jenifer Delgado Rich
// jedelrich@hotmail.com
// Odometria sacada de datos de GPS diferencial (< 2cm de error). El (0,0,0) del eje de referencia es la
// primera posicion que se obtiene del GPS y este eje de referencia esta orientado en Este --> X, Norte --> Y

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher path;

nav_msgs::Path msg_path;

void gps_odom_callback(const nav_msgs::Odometry& msg)
{
    geometry_msgs::PoseStamped m;

    m.pose.position.x = msg.pose.pose.position.x;
    m.pose.position.y = msg.pose.pose.position.y;
    m.pose.position.z = msg.pose.pose.position.z;

    msg_path.poses.push_back(m);
    path.publish(msg_path);
    msg_path.header.seq++;
}


int main ( int argc, char** argv )
{
    ros::init (argc, argv, "gps_path");
    ros::NodeHandle n;
    msg_path.header.frame_id = "world";
    
    ros::Subscriber sub = n.subscribe("/odometry_ground_truth", 10, gps_odom_callback);
    path = n.advertise<nav_msgs::Path>("/gps/path", 100); 

    ros::spin();
    return 0;
}
