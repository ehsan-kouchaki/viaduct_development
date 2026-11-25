/**
*   @file       direct_hough.cpp
*
*   @author     Juan Antonio Sanchez Diaz <juanantoniodiaz99@gmail.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Main node of laser localization using hough transform in the viaduct scenarios
*/

#include "DirectHough.h"

/**
      * @brief Main function
      * @param argc argument count 
      * @param argv character array that holds all the arguments
      * @return exit of program is int variable
*/

int main (int argc, char **argv)
{
  std::string _node_name = "lidar_localization";
  ros::init(argc, argv, _node_name);
  ros::NodeHandle n;

  ROS_INFO("[%s]: node initialized", _node_name.c_str());

  // Check is roscore is running
  if (!ros::master::check())
  {
      ROS_ERROR("[%s]: roscore is not running.", _node_name.c_str());
      return 0;
  }

	// Filter parameters
	n.getParam(_node_name + "/ANGLE_MAX",max);	// Angulo maximo para filtrado
	n.getParam(_node_name + "/ANGLE_MIN",min);	// Angulo minimo para filtrado
	n.getParam(_node_name + "/MAX_INTENSITY",intensity);  // Minima intensidad aceptada
	n.getParam(_node_name + "/MAX_DISTANCE",distance);  // Distancia maxima aceptada
  n.getParam(_node_name + "/DIS2LINE",d);  // Distancia maxima aceptada
  n.getParam(_node_name + "/MIN_POINTS_LINE",min_points);
	n.getParam(_node_name + "/SCAN_TOPIC",scan_topic);
  n.getParam(_node_name + "/ODOM_FRAME_ID",odom_frame_id);
  n.getParam(_node_name + "/ROT_XY",XY_hough);
  n.getParam(_node_name + "/ROT_XZ",XZ_hough);
  n.getParam(_node_name + "/PATH_FRAME_ID",XZ_hough);

  tf::TransformBroadcaster *tfbr = new tf::TransformBroadcaster;
  // Create objet of type KFLocalization and launch ROS spin loop
  DirectHough dhf(n, tfbr);
  dhf.spin();

  // Report that the node has been finished
  ROS_INFO("[%s]: node finished", node_name.c_str());

  return 0;
}

