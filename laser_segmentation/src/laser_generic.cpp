/**
*   @file       laser_generic.cpp
*
*   @author     Juan Antonio Sanchez Diaz <jsanchez22@us.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Main node of laser localization by data segmentation in viaduct scenarios
**/

#include "laser.h"

/**
*   @brief Main function
*   @param argc argument count 
*   @param argv character array that holds all the arguments
*   @return exit of program is int variable
**/

int main (int argc, char **argv)
{
  ROS_INFO("Starting Localization node with lidar info...");
  std::string _node_name = "laser_generic";
  ros::init(argc, argv, _node_name);
  ros::NodeHandle n;

  // Check is roscore is running
  if(!ros::master::check())
  {
    ROS_ERROR("[%s]: roscore is not running.", _node_name.c_str());
    return 0;
  }

  // Create objet of type DirectHough and launch ROS spin loop
  LaserGeneric LG(n, _node_name);
  //LG.spin();

  while(true){   
    ros::Duration(0.03).sleep();
    LG.spinOnce();
  }

  // Report that the node has been finished
  ROS_INFO("[%s]: node finished", _node_name.c_str());

  return 0;
}