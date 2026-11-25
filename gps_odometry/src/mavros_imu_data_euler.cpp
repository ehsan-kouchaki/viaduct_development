#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

ros::Publisher pos_angle_pub;
void imu_cb(const sensor_msgs::Imu msg){

  geometry_msgs::Quaternion q_imu;
  q_imu = msg.orientation;
  tf::Quaternion q_mavros(q_imu.x, q_imu.y, q_imu.z, q_imu.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q_mavros).getRPY(roll, pitch, yaw);

  geometry_msgs::PoseStamped position_angle;
  position_angle.header = msg.header;
  position_angle.pose.position.x = roll*180/M_PI;
  position_angle.pose.position.y = pitch*180/M_PI;
  position_angle.pose.position.z = yaw*180/M_PI;

  pos_angle_pub.publish(position_angle);

}

int main ( int argc, char** argv )
{
    ros::init ( argc, argv, "mavros_imu_data_euler" );
    ros::NodeHandle node("~");

    pos_angle_pub = node.advertise<geometry_msgs::PoseStamped>("/mavros/imu_euler", 1);

    ros::Subscriber sub_imu = node.subscribe("/mavros/imu/data", 1, imu_cb);

    ros::spin();

    return 0;
}
