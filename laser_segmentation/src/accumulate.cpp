#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<nav_msgs/Odometry.h>

//PUBLISHER
ros::Publisher pub;

//GLOBALS MSGS
sensor_msgs::LaserScan acc_msg;

int cont = 0;

void LaserCallback(const  sensor_msgs::LaserScan::ConstPtr& msg)
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
  for (int k = 0; k < hor_laser.ranges.size(); k++) {
    if (k > min && k < max) {
        if (hor_laser.ranges[k] == std::numeric_limits<float>::infinity() || hor_laser.ranges[k] > 10000) {
            hor_laser.ranges[k] = 0.0;
            ver_laser.ranges[k] = 0.0;
        }
    } else {
        ver_laser.ranges[k] = 0.0;
    }
    if (hor_laser.ranges[k] >= distance || (k < max + offset + 100 && k > min - offset - 100) || hor_laser.ranges[k] <= 1) {
        hor_laser.ranges[k] = 0.0;
    }
  }
  
  acc_msg.ranges.insert(acc_msg.ranges.end(), hor_laser.ranges.begin(), hor_laser.ranges.end());
  cont++;

  if(cont == 3)
  {
    acc_msg.header = msg->header;
    acc_msg.angle_min = msg->angle_min;
    acc_msg.angle_max = msg->angle_max;
    acc_msg.angle_increment = msg->angle_increment;
    acc_msg.time_increment = msg->time_increment;
    acc_msg.scan_time = msg->scan_time;
    acc_msg.range_min = msg->range_min;
    acc_msg.range_max = msg->range_max;
    acc_msg.ranges.insert(acc_msg.ranges.end(), ver_laser.ranges.begin(), ver_laser.ranges.end());
    pub.publish(acc_msg);

    acc_msg.ranges.clear();
    cont = 0;
  }
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "accumulate_pointcloud");
  ros::NodeHandle n;

  //Publishers
  pub = n.advertise<sensor_msgs::LaserScan>("/acc_scan", 100); 

  //Subscribers
  ros::Subscriber laser = n.subscribe("/scan", 100, LaserCallback);

  //waiting messages
  ros::spin();

  return 0;
}
