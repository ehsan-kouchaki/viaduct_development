/**
*   @file       viaduct_kf_localization.cpp
*
*   @author     Maria Polvillo Diaz <mapodi.94@gmail.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Main node of KF localization filter in the viaduct scenarios
*/

#include "ViaductKFLocalization.h"

/**
      * @brief Main function
      * @param argc argument count 
      * @param argv character array that holds all the arguments
      * @return exit of program is int variable
*/
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "viaduct_kf_localization");
    ros::NodeHandle nh("~");
    std::string node_name = ros::this_node::getName();
    ROS_INFO("[%s]: node initialized", node_name.c_str());

    // Check is roscore is running
    if (!ros::master::check())
    {
        ROS_ERROR("[%s]: roscore is not running.", node_name.c_str());
        return 0;
    }

    // Create objet of type KFLocalization and launch ROS spin loop
    tf::TransformBroadcaster *tfbr = new tf::TransformBroadcaster;
    KFLocalization kf(nh, tfbr);
    kf.spin();

    // Report that the node has been finished
    ROS_INFO("[%s]: node finished", node_name.c_str());

    return 0;
}
