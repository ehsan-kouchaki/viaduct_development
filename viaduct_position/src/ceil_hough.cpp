#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include "laser_geometry/laser_geometry.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "mission_manager/mission_manager.h"
#include <tf/transform_broadcaster.h>

laser_geometry::LaserProjection projector_; // Proyector de puntos en el espacio

// PUBLISHER
ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub2d;
ros::Publisher path;
ros::Publisher high;
ros::Publisher houghp;
nav_msgs::Path msg_path;
geometry_msgs::PoseStamped m;
geometry_msgs::PoseStamped mz;

// GLOBALS MSGS
sensor_msgs::PointCloud data;
sensor_msgs::PointCloud data_mirror;

// LASER FILTER PARAMETERS
int max = 271;
int min = 220;
float distance = 9000;
float aux = 0;
float d = 1;
int min_points = 10;
std::string scan_topic = "scan";
std::string path_frame_id = "laser";
std::string laser_frame_id = "laser";
std::string odom_topic = "hough/ceil";

// POSITION
float xa = 0, ya = 0, za = 0, thetaa = 0;
float xaa = 0, yaa = 0, zaa = 0;
float xi = 0, yi = 0, zi = 0, thetai = 0;
float XG = 0, YG = 0, ZG = 0, YAWG = 0;
float error = 5;
bool ini = true;
bool takeoff = false;
bool debug = false;

float yaw = 0;

float d_roofa = 0;
int cont_med = 0;

unsigned int* _accu;
int ir_max_a = 0, ir_max_a2 = 0;
int it_max_a = 0;

float XY_hough = 0, XZ_hough = 0; 

void lidar_callback(const  sensor_msgs::LaserScan::ConstPtr& msg)
{
  if(takeoff)
  {
    // MEDIR TIEMPO DE COMPUTO pcl o ros:time
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();

    sensor_msgs::LaserScan msg_laser = *msg;
    
    size_t longitud = msg->ranges.size(); //Vector's lenght

    int ofs = 100;
    max = round(longitud/2) + ofs; //433
    min = round(longitud/2) - ofs; //292
    for(int k = 0; k < msg_laser.ranges.size(); k++) //Estudio los puntos del mensaje y los distingo
    {
      if(msg_laser.ranges[k] == std::numeric_limits<float>::infinity()) msg_laser.ranges[k] = 0.0;

      // Mirror data
      if(!(k > min && k < max))
      {
        msg_laser.ranges[k] = 0.0;
      } 
    }

    msg_laser.header.frame_id = laser_frame_id;

    //Updating the new msgs
    projector_.projectLaser(msg_laser, data);
    data.header.frame_id = laser_frame_id;
    pub.publish(data);

    //######################## MIRROR HOUGH TRANSFORM ###############################
    int escala = 100; //Resolucion en rho
    int dist = distance; //Distancia maxima de las rectas
    int i_R = dist*escala;
    int i_T = 360;

    _accu = (unsigned int*)calloc(i_R * i_T, sizeof(unsigned int));

    double center_x = 0;
    double center_y = 0;

    // Rellenamos matriz de votos de Hough
    for(int i = 0; i<data.points.size(); i++)
    {
      float x = data.points[i].x;
      float y = data.points[i].y;
      for(int t=0;t<i_T;t++)
      {
        float r = ((x - center_x) * cos((double)t * CV_PI/180)) + (((double)y - center_y) * sin((double)t * CV_PI/180));
        r = abs(round(r*escala));
        _accu[(int)((r * i_T)) + t]++;
      }
    }
    
    int ir_max = 0;
    int it_max = 0;
    int max = 10;
    //std::cout << ir_max_a << std::endl;
    if(!ini)
    {
      for(int r=ir_max_a-200;r<ir_max_a+200;r++)
      {
        for(int t=it_max_a-10;t<it_max_a+10;t++) // Cuidado con los giros
        {
          if (_accu[(int)((r * i_T)) + t] > max)
          {
            max = _accu[(int)((r * i_T)) + t];
            ir_max = r;
            it_max = t;
          }
        }
      } 
    }
    else // Buscar cerca del punto anterior
    {
      for(int r=0;r<i_R;r++)
      {
        for(int t=160;t<200;t++)
        {
          if (_accu[(int)((r * i_T)) + t] > max) // Numero minimo de puntos para deteccion
          {
            max = _accu[(int)((r * i_T)) + t];
            ir_max = r;
            it_max = t;
          }
        }
      }
    }

    if (debug)
    {
      std::cout << "-----------------------------------------------------------" << std::endl;
      std::cout << "max = " << max << ", ir_max = " << ir_max << ", it_max = " << it_max << std::endl;
      std::cout << "rho = " << ir_max/(float)escala << std::endl;
      std::cout << "-----------------------------------------------------------" << std::endl;
    }

    if(ir_max > 0) // Hay deteccion de linea
    {
      ir_max_a = ir_max;
      it_max_a = it_max;
      float rho = ir_max/(float)escala;
      int theta = it_max;

      float m00 = 0, m01 = 0, m10 = 0;
      for (int i = 0; i < data.points.size(); i++)
      {
        float yi = data.points[i].x;
        float xi = data.points[i].y;
        if (theta*CV_PI/180 != 0)
        {
          float yl = rho ; //- xi*cos(theta*CV_PI/180)) / sin(theta*CV_PI/180)

          if (abs((yi-yl)) < d) 
          {
            m00 += 1;
            m10 += xi;
            m01 += yi;
          }
        }
      }

      if (debug) std::cout << "m00 = " << m00 << std::endl;

      if (m00>0)
      {
        float xCM = m01/m00;
        float yCM = m10/m00;

        pcl::PointCloud<pcl::PointXYZI> clouds1_pcl;
        pcl::PointXYZI point;
        point.x = xCM;
        point.y = yCM;
        point.z = 0;
        point.intensity = 0;
        clouds1_pcl.push_back(point);

        sensor_msgs::PointCloud2 output_ros;
        pcl::toROSMsg(clouds1_pcl, output_ros);

        output_ros.header = msg_laser.header;
        
        pub2d.publish(output_ros);

        if (!ini)
        {
          theta = (theta + thetaa) / 2;
          yaw = -(theta - thetai);
          
          m.pose.position.x =  - ((point.x*cos(yaw*CV_PI/180) - point.y*sin(yaw*CV_PI/180)) - xi); 
          m.pose.position.y = - ((point.x*sin(yaw*CV_PI/180) + point.y*cos(yaw*CV_PI/180)) - yi);

          // print results
          end_ = ros::WallTime::now();
          double execution_time = (end_ - start_).toNSec() * 1e-6;

          if (debug)
          {
            std::cout << "Relative position: " << std::endl;
            std::cout << "x = " << point.x << ", y = " << point.y << ", theta = " << theta*CV_PI/180 << ", rho = " << rho << std::endl;
            std::cout << "xt = " << (xi*cos(yaw*CV_PI/180) - yi*sin(yaw*CV_PI/180)) << ", yt = " << (xi*sin(yaw*CV_PI/180) + yi*cos(yaw*CV_PI/180)) << std::endl;
            std::cout << "Xg = " << m.pose.position.x << ", Yg = " << m.pose.position.y << ", yawg = " << yaw*CV_PI/180 << std::endl;
            std::cout << "Execution time: " << execution_time << std::endl;
            std::cout << "-----------------------------------------------------------" << std::endl; 
          }
          
          m.pose.position.x = m.pose.position.x;
          m.pose.position.y = m.pose.position.y;

          //Suavizado con muestras anteriores
          if (cont_med >= 3)
          {
            m.pose.position.x = (m.pose.position.x + xa + xaa) / 3;
            m.pose.position.y = (m.pose.position.y + ya + yaa) / 3;
          } 

          if(abs(m.pose.position.x - xa) < error && abs(m.pose.position.y - ya) < error)
          {
            if (cont_med < 3) cont_med++;
            // Almacenamos los valores anteriores
            xaa = xa;
            yaa = ya;
            zaa = za;
            xa = m.pose.position.x;
            ya = m.pose.position.y;
            za = m.pose.position.z;
            thetaa = theta;

            float xrot = 0;
            float yrot = 0;
            float zrot = 0; 

            xrot = m.pose.position.x;
            yrot = m.pose.position.y;
            zrot = m.pose.position.z;

            // XY Rotation
            float xtrans = xrot;
            float ytrans = yrot;
            float alpha = XY_hough * 3.1416/180; 
            xrot = xtrans*cos(alpha) + ytrans*sin(alpha);
            yrot = -xtrans*sin(alpha) + ytrans*cos(alpha); 

            // XZ Rotation
            xtrans = xrot;
            float ztrans = zrot;
            alpha = XZ_hough * 3.1416/180;
            xrot = xtrans*cos(alpha) + ztrans*sin(alpha);
            zrot = -xtrans*sin(alpha) + ztrans*cos(alpha);

            m.pose.position.x = xrot;
            m.pose.position.y = yrot;
            m.pose.position.z = zrot;

            tf::Quaternion q_t = tf::createQuaternionFromRPY(0, 0, yaw*CV_PI/180); //- XY_hough*CV_PI/180
            geometry_msgs::Quaternion _q_pose;
            _q_pose.x = q_t.x();
            _q_pose.y = q_t.y();
            _q_pose.z = q_t.z();
            _q_pose.w = q_t.w();
            m.pose.orientation = _q_pose;
            
            nav_msgs::Odometry odom;
            odom.pose.pose = m.pose;
            odom.header.frame_id = path_frame_id;
            houghp.publish(odom);
          }
        }
        else
        {
          xi = point.x;
          yi = point.y;
          thetai = theta;
          thetaa = thetai;
          ini = false;

          if (debug)
          {
            std::cout << "-----------------------------------------------------------" << std::endl;
            std::cout << "---------------- Transformation analisys ------------------" << std::endl;
            std::cout << "-----------------------------------------------------------" << std::endl;
            std::cout << "Reference position: " << std::endl;
            std::cout << "xi = " << xi << ", yi = " << yi << ", thetai = " << thetai*CV_PI/180 << ", rhoi = " << rho << std::endl;
            std::cout << "-----------------------------------------------------------" << std::endl;
          }
        }
      }
    }
  }
}

int main (int argc, char **argv)
{
  std::string _node_name = "ceil_pose_adjustment";
  ros::init(argc, argv, _node_name);
  ros::NodeHandle n;

  ROS_INFO("Launching node...");

	// Filter parameters
	n.getParam(_node_name + "/ANGLE_MAX",max);	// Angulo maximo para filtrado
	n.getParam(_node_name + "/ANGLE_MIN",min);	// Angulo minimo para filtrado
	n.getParam(_node_name + "/MAX_DISTANCE",distance);  // Distancia maxima aceptada
  n.getParam(_node_name + "/DIS2LINE",d);  // Distancia maxima aceptada
  n.getParam(_node_name + "/MIN_POINTS_LINE",min_points);
	n.getParam(_node_name + "/SCAN_TOPIC",scan_topic);
  n.getParam(_node_name + "/LASER_FRAME_ID",laser_frame_id);
  n.getParam(_node_name + "/ROT_XY",XY_hough);
  n.getParam(_node_name + "/ROT_XZ",XZ_hough);
  n.getParam(_node_name + "/PATH_FRAME_ID",path_frame_id);
  n.getParam(_node_name + "/ODOMETRY_TOPIC",odom_topic);
  n.getParam(_node_name + "/ACCEPTED_ERROR",error);

  //Publishers
  pub2d = n.advertise<sensor_msgs::PointCloud2>("/laser2d", 100);
  pub = n.advertise<sensor_msgs::PointCloud>("/laser_filtered", 100);
  houghp = n.advertise<nav_msgs::Odometry>(odom_topic, 100);

  //Subscribers
  ros::Subscriber laser = n.subscribe(scan_topic, 100, lidar_callback);

  msg_path.header.frame_id = path_frame_id; 

  ROS_INFO("Waiting data...");

  //waiting messages
  ros::spin();

  return 0;
}
