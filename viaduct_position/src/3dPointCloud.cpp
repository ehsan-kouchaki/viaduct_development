#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<nav_msgs/Odometry.h>
#include "laser_geometry/laser_geometry.h"

// rosrun pcl_ros pcd_to_pointcloud alora_2021_03_11.pcd 0.1 _frame_id:=/odom

laser_geometry::LaserProjection projector_;

//PUBLISHER
ros::Publisher pubh;
ros::Publisher pubv;

//GLOBALS MSGS
sensor_msgs::PointCloud hor;
sensor_msgs::PointCloud ver;

void ReadData(const  sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan hor_laser;
  sensor_msgs::LaserScan ver_laser;

  size_t longitud = msg->ranges.size(); //Longitud del vector

  hor_laser = *msg;
  ver_laser = *msg;
  
  //Realizamos la separacion de puntos vertical y horizontal
  int offset = 100;
  int distance = 9000;
  int max = round(longitud/2) + offset; //433
  int min = round(longitud/2) - offset; //292

  for(int k = 0; k < hor_laser.ranges.size(); k++) //Estudio los puntos del mensaje y los distingo
  {
    if(hor_laser.ranges[k] == std::numeric_limits<float>::infinity()) hor_laser.ranges[k] = 0.0;

    // Mirror data
    if(k > min && k < max)
    {
      if(hor_laser.ranges[k] == 0.0 && hor_laser.ranges[k] > 10000)  //hor_laser.ranges[k] < zaux && 
      {
        ver_laser.ranges[k] = 0.0;
      }
      hor_laser.ranges[k] = 0.0;
    } 
    else
    {
      ver_laser.ranges[k] = 0.0;
    }

    if(hor_laser.ranges[k] >= distance || (k < max + offset + 100 && k > min - offset - 100))
    {
      hor_laser.ranges[k] = 0.0;
    } 
  }

  // for(int k = 0; k < longitud; k++) //Estudio los puntos del mensaje y los distingo
  // {
  //   if (k < min || k > max) ver_laser.ranges[k] = 0;
  //   if (k > min && k < max) hor_laser.ranges[k] = 0;
  //   if (hor_laser.ranges[k] == std::numeric_limits<float>::infinity()) hor_laser.ranges[k] = 0;
  //   if (ver_laser.ranges[k] == std::numeric_limits<float>::infinity()) ver_laser.ranges[k] = 0;
  // }

    //Updating the new msgs
  projector_.projectLaser(hor_laser, hor);
  projector_.projectLaser(ver_laser, ver);

  longitud = ver.points.size();

  for(int k = 0; k< longitud; k++)
  {
    ver.points[k].z = ver.points[k].x;
    ver.points[k].x = 0;
  }

  sensor_msgs::PointCloud laser_3dh;
  sensor_msgs::PointCloud laser_3dv;

  laser_3dh.header = hor.header;
  laser_3dh.points.insert(laser_3dh.points.end(),std::begin(hor.points),std::end(hor.points));
  laser_3dv.header = ver.header;
  laser_3dv.points.insert(laser_3dv.points.end(),std::begin(ver.points),std::end(ver.points));
  
  laser_3dv.header.frame_id = "laser";
  laser_3dh.header.frame_id = "laser";
  
  pubh.publish(laser_3dh);
  pubv.publish(laser_3dv);
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "Read_Laser");
  ros::NodeHandle n;

  //Publishers
  pubh = n.advertise<sensor_msgs::PointCloud>("/3d_hor", 100); 
  pubv = n.advertise<sensor_msgs::PointCloud>("/3d_ver", 100); 

  //Subscribers
  ros::Subscriber laser = n.subscribe("/scan", 100, ReadData);

  //waiting messages
  ros::spin();

  return 0;
}
