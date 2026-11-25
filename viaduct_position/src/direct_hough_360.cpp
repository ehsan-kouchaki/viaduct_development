/**
*   @file       direct_hough_360.cpp
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

// POLAR TO CARTESIAN COORDINATES
laser_geometry::LaserProjection projector_; // Proyector de puntos en el espacio

// PUBLISHER
ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub2d;
ros::Publisher path;
ros::Publisher high;
ros::Publisher houghp;

// LOCALIZATION
nav_msgs::Path msg_path;
geometry_msgs::PoseStamped m;
geometry_msgs::PoseStamped mz;
std_msgs::Float32 ceil_distance;

// TOPIC'S NAME
std::string scan_topic = "/scan";
std::string filtered_laser_topic = "/laser_filtered";
std::string filtered_mirror_topic = "/laser_filtered_mirror";
std::string path_topic = "/hough/path";
std::string pose_topic = "/hough/pose";
std::string z_topic = "/hough/ceil";
std::string mission_manager_service = "mission_manager_hough";

// FRAME ID
std::string path_frame_id = "world";
std::string laser_frame_id = "laser";

// GLOBALS MSGS
sensor_msgs::PointCloud data;
sensor_msgs::PointCloud data_mirror;

// LASER FILTER PARAMETERS
int max = 271;
int min = 220;
float distance = 9000;
float aux = 0;
float d = 0.2;
int min_points = 10;
int ofs = 100;

// POSITION
float xa = 0, ya = 0, za = 0, thetaa = 0;
float xaa = 0, yaa = 0, zaa = 0, thetaaa = 0;
float xi = 0, yi = 0, zi = 0, thetai = 0;
float XG = 0, YG = 0, ZG = 0, YAWG = 0;
float error = 2000;

float yaw = 0;

float d_roofa = 0;
int cont_med = 0;

unsigned int* _accu;
int ir_max_a = 0, ir_max_a2 = 0;
int it_max_a = 0, it_max_a2 = 0;

float XY_hough = 0, XZ_hough = 0; 

// SETTINGS
bool ini = true;
bool takeoff = false;
bool debug = false;

void lidar_callback(const  sensor_msgs::LaserScan::ConstPtr& msg)
{
  if(!takeoff)
  {
    // MEDIR TIEMPO DE COMPUTO pcl o ros:time
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();

    sensor_msgs::LaserScan msg_laser = *msg;
    sensor_msgs::LaserScan mirror_laser = *msg;
    
    size_t longitud = msg->ranges.size(); //Vector's lenght

    max = round(longitud/2) + ofs; //433
    min = round(longitud/2) - ofs; //292
    float zaux = 0;
    float total = 0;
    for (int k = 0; k < msg_laser.ranges.size(); k++){
      if (msg_laser.ranges[k] == std::numeric_limits<float>::infinity()){
        msg_laser.ranges[k] = 0.0;
      }
      if (k > min && k < max){
          if(msg_laser.ranges[k] != 0.0 && msg_laser.ranges[k] <= 10000){
            zaux += msg_laser.ranges[k];
            total++;
          }else{
            msg_laser.ranges[k] = 0.0;
            mirror_laser.ranges[k] = 0.0;
          }
      }else{
        mirror_laser.ranges[k] = 0.0;
      }
      if (msg_laser.ranges[k] >= distance || (k < max + ofs + 100 && k > min - ofs - 100)){
        msg_laser.ranges[k] = 0.0;
      }
    }

    zaux = zaux / total;

    msg_laser.header.frame_id = laser_frame_id;

    //Updating the new msgs
    projector_.projectLaser(msg_laser, data);
    data.header.frame_id = laser_frame_id;
    pub.publish(data);

    projector_.projectLaser(mirror_laser, data_mirror);
    pub2.publish(data_mirror);
    //std::cout << "debug3" << std::endl;

    //######################## MIRROR HOUGH TRANSFORM ###############################
    int escala = 100; //Resolucion en rho
    int dist = distance; //Distancia maxima de las rectas
    int i_R = dist*escala;
    int i_T = 360;
    
    _accu = (unsigned int*)calloc(i_R * i_T, sizeof(unsigned int));

    double center_x = 0;
    double center_y = 0;

    // Rellenamos matriz de votos de Hough
    for(int i = 0; i<data_mirror.points.size(); i++)
    {
      float x = data_mirror.points[i].y;
      float y = data_mirror.points[i].x;
      for(int t=0;t<i_T;t++)
      {
        float r = ((x - center_x) * cos((double)t * CV_PI/180)) + (((double)y - center_y) * sin((double)t * CV_PI/180));
        r = abs(round(r*escala));
        _accu[(int)((r * i_T)) + t]++;
      }
    }

    //std::cout << "debug2" << std::endl;

    int ir_max = 0;
    int it_max = 0;
    int max = 0;
    if(!ini){
      int ran_min = ir_max_a2-30;
      if (ran_min < 0) ran_min = 0;
      //std::cout << ran_min << " " << it_max_a2-10 << std::endl;
      for(int r=ran_min;r<ir_max_a2+30;r++){
        // TODO: COMPROBAR it_max_a2-10 > 0
        for(int t=90-20;t<90+20;t++){ // Cuidado con los giros // it_max_a2-10;t<it_max_a2+10
          if(it_max_a2-10 >= 0){
            if(_accu[(int)((r * i_T)) + t] > max){
              //std::cout << r << " " << t << std::endl;
              max = _accu[(int)((r * i_T)) + t];
              ir_max = r;
              it_max = t;
            }
          } 
        }
      }
      if (abs(it_max_a2 - it_max) < 20) it_max_a2 = it_max;
      //else {it_max = it_max_a2;}
      if (abs(ir_max_a2 - ir_max) < 20) ir_max_a2 = ir_max;
      //else {ir_max = ir_max_a2;}
    }else{ // Buscar cerca del punto anterior
      for(int r=0;r<i_R;r++){
        for(int t=0;t<i_T;t++){
          if (_accu[(int)((r * i_T)) + t] > max){ // Numero minimo de puntos para deteccion
            max = _accu[(int)((r * i_T)) + t];
            ir_max = r;
            it_max = t;
          }
        }
      }
      it_max_a2 = it_max;
      ir_max_a2 = ir_max;
    }

    if(debug){
      std::cout << "-----------------------MIRROR------------------------------" << std::endl;
      std::cout << "max = " << max << ", ir_max = " << ir_max << ", it_max = " << it_max << std::endl;
      std::cout << "-----------------------------------------------------------" << std::endl;
    }


    //std::cout << "debug" << std::endl;

    float d_roof = abs((ir_max - i_R)/(float)escala);
    ceil_distance.data = ir_max/(float)escala - 0.14;
    high.publish(ceil_distance);

    /*if(!ini)
    {
      if(abs(d_roof - d_roofa) < 0.5)
      {
        d_roofa = d_roof;
      }
      else
      {
        d_roof = d_roofa;
      }
    }*/

    //######################## HOUGH TRANSFORM ###############################
    /* escala = 100; //Resolucion en rho
    dist = distance; //Distancia maxima de las rectas
    i_R = dist*escala;
    i_T = 360; */

    free(_accu);

    _accu = (unsigned int*)calloc(i_R * i_T, sizeof(unsigned int));

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
    
    ir_max = 0;
    it_max = 0;
    max = 5;
    if(!ini)
    {
      int ran_min = ir_max_a-300;
      if (ran_min < 0) ran_min = 0;
      for(int r=ran_min;r<ir_max_a+200;r++)
      {
        for(int t=160;t<200;t++) // Cuidado con los giros int t=it_max_a-10;t<it_max_a+10;t++
        {
          //std::cout << r << " " << t << std::endl;
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
      for(int r=10;r<i_R ;r++) // reducimos rango de busqueda en la primera iteracion => - distance/2*escala // 1500
      {
        for(int t=160;t<200;t++)
        {
          if (_accu[(int)((r * i_T)) + t] > max) // Numero minimo de puntos para deteccion
          {
            // if(t > 200 || t < 160)
            // { 
            //   std::cout << "-----------------------------------------------------------" << std::endl;
            //   std::cout << "r = " << r << ", t = " << t << std::endl;
            //   std::cout << "-----------------------------------------------------------" << std::endl;
            // }
            max = _accu[(int)((r * i_T)) + t];
            ir_max = r;
            it_max = t;
          }
        }
      }
    }

    if(debug)
    {
      std::cout << "-----------------------------------------------------------" << std::endl;
      std::cout << "max = " << max << ", ir_max = " << ir_max << ", it_max = " << it_max << std::endl;
      std::cout << "-----------------------------------------------------------" << std::endl;
    }

    if(ir_max > 0) // Hay deteccion de linea
    {
      ir_max_a = ir_max;
      it_max_a = it_max;
      int theta = it_max;
      float rho = ir_max/(float)escala;

      float m00 = 0, m01 = 0, m10 = 0;
      for (int i = 0; i < data.points.size(); i++)
      {
        float xi = data.points[i].x;
        float yi = data.points[i].y;
        if (theta*CV_PI/180 != 0)
        {
          float yl = (rho - xi*cos(theta*CV_PI/180)) / sin(theta*CV_PI/180);
          if (abs((yi-yl)*sin(theta*CV_PI/180)) < d)
          {
            m00 += 1;
            m10 += xi;
            m01 += yi;
          }
        }
        else
        {
          if (abs(xi - rho) < d)
          {
            m00 += 1;
            m10 += xi;
            m01 += yi;
          }
        }
      }

      if (m00>0)
      {
        float xCM = m10/m00;
        float yCM = m01/m00;

        // Comprobar cercania de puntos a la media y filtrar y volver a calcular media
        // m00 = 0, m01 = 0, m10 = 0;
        // for (int i = 0; i < data.points.size(); i++)
        // {
        //   float xi = data.points[i].x;
        //   float yi = data.points[i].y;
        //   float dCM = sqrt(pow((xi-xCM),2) + pow((yi-yCM),2));
        //   if (theta*CV_PI/180 != 0)
        //   {
        //     float yl = (rho - xi*cos(theta*CV_PI/180)) / sin(theta*CV_PI/180);
        //     if (abs((yi-yl)*sin(theta*CV_PI/180)) < d && dCM < 6)
        //     {
        //       m00 += 1;
        //       m10 += xi;
        //       m01 += yi;
        //     }
        //   }
        //   else
        //   {
        //     if (abs(xi - rho) < d && dCM < 4)
        //     {
        //       m00 += 1;
        //       m10 += xi;  
        //       m01 += yi;
        //     }
        //   }
        // }

        std::vector<std::vector<double>> points(2, std::vector<double>(2)); 
        double dist = 0.0;
        // primer punto
        for (int i = 0; i < data.points.size(); i++)
        {
          float xi = data.points[i].x;
          float yi = data.points[i].y;
          float dCM = sqrt(pow((xi-xCM),2) + pow((yi-yCM),2));
          if (theta*CV_PI/180 != 0)
          {
            float yl = (rho - xi*cos(theta*CV_PI/180)) / sin(theta*CV_PI/180);
            if (abs((yi-yl)*sin(theta*CV_PI/180)) < d && dCM < 6)
            {
              double norma = sqrt(pow(xCM - xi, 2) + pow(yCM - yi, 2));
              if (norma > dist){
                dist = norma;
                points[0][0] = xi;
                points[0][1] = yi;
              }
            }
          }
          else
          {
            if (abs(xi - rho) < d && dCM < 4)
            {
              double norma = sqrt(pow(xCM - xi, 2) + pow(yCM - yi, 2));
              if (norma > dist){
                dist = norma;
                points[0][0] = xi;
                points[0][1] = yi;
              }
            }
          }
        }

        dist = 0.0;
        // segundo punto
        for (int i = 0; i < data.points.size(); i++)
        {
          float xi = data.points[i].x;
          float yi = data.points[i].y;
          float dCM = sqrt(pow((xi-xCM),2) + pow((yi-yCM),2));
          if (theta*CV_PI/180 != 0)
          {
            float yl = (rho - xi*cos(theta*CV_PI/180)) / sin(theta*CV_PI/180);
            if (abs((yi-yl)*sin(theta*CV_PI/180)) < d && dCM < 6)
            {
              double norma = sqrt(pow(points[0][0] - xi, 2) + pow(points[0][1] - yi, 2));
              if (norma > dist){
                dist = norma;
                points[1][0] = xi;
                points[1][1] = yi;
              }
            }
          }
          else
          {
            if (abs(xi - rho) < d && dCM < 4)
            {
              double norma = sqrt(pow(points[0][0] - xi, 2) + pow(points[0][1] - yi, 2));
              if (norma > dist){
                dist = norma;
                points[1][0] = xi;
                points[1][1] = yi;
              }
            }
          }
        }

        xCM = (points[0][0] + points[1][0]) / 2;
        yCM = (points[0][1] + points[1][1]) / 2;

        pcl::PointCloud<pcl::PointXYZI> clouds1_pcl;
        pcl::PointXYZI data;
        for(auto point:points) 
        {
          data.x = point[0];
          data.y = point[1];
          data.z = 0;
          data.intensity = 0;
          clouds1_pcl.push_back(data);
        }

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
          
          /* m.pose.position.x = point.x - (xi*cos(yaw*CV_PI/180) - yi*sin(yaw*CV_PI/180)); 
          m.pose.position.y = point.y - (xi*sin(yaw*CV_PI/180) + yi*cos(yaw*CV_PI/180)); */
          
          m.pose.position.x = (point.x*cos(yaw*CV_PI/180) - point.y*sin(yaw*CV_PI/180)) - xi; 
          m.pose.position.y = (point.x*sin(yaw*CV_PI/180) + point.y*cos(yaw*CV_PI/180)) - yi;
          m.pose.position.z = d_roof - zi + sqrt(0.05*abs(m.pose.position.x));

          // print results
          end_ = ros::WallTime::now();
          double execution_time = (end_ - start_).toNSec() * 1e-6;

          if (debug)
          {
            std::cout << "Relative position: " << std::endl;
            std::cout << "x = " << point.x << ", y = " << point.y << ", theta = " << theta*CV_PI/180 << ", rho = " << rho << std::endl;
            std::cout << "xt = " << (xi*cos(yaw*CV_PI/180) - yi*sin(yaw*CV_PI/180)) << ", yt = " << (xi*sin(yaw*CV_PI/180) + yi*cos(yaw*CV_PI/180)) << std::endl;
            std::cout << "Xg = " << m.pose.position.x << ", Yg = " << m.pose.position.y << ", yawg = " << yaw*CV_PI/180 << std::endl;
            std::cout << "xCM = " << xCM << ", yCM = " << yCM << std::endl;
            std::cout << "Execution time: " << execution_time << std::endl;
            std::cout << "-----------------------------------------------------------" << std::endl;
          }
          
          m.pose.position.x = m.pose.position.x + XG;
          m.pose.position.y = m.pose.position.y + YG;
          m.pose.position.z = m.pose.position.z + ZG;

          //Suavizado con muestras anteriores
          if (cont_med >= 3)
          {
            m.pose.position.x = (m.pose.position.x + xa + xaa) / 3;
            m.pose.position.y = (m.pose.position.y + ya + yaa) / 3;
            m.pose.position.z = (m.pose.position.z + za) / 2;
          } 

          if(abs(m.pose.position.x - xa) < error && abs(m.pose.position.y - ya) < error && abs(m.pose.position.z - za) < error)
          {
            if (cont_med < 3) cont_med++;
            // Almacenamos los valores anteriores
            xaa = xa;
            yaa = ya;
            zaa = za;
            thetaaa = thetaa;
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

            tf::Quaternion q_t = tf::createQuaternionFromRPY(0, 0, yaw*CV_PI/180 + YAWG); //- XY_hough*CV_PI/180
            geometry_msgs::Quaternion _q_pose;
            _q_pose.x = q_t.x();
            _q_pose.y = q_t.y();
            _q_pose.z = q_t.z();
            _q_pose.w = q_t.w();
            m.pose.orientation = _q_pose;

            //std::cout << _q_pose << std::endl; 

            msg_path.poses.push_back(m);
            path.publish(msg_path);
            msg_path.header.seq++;
            
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
          zi = abs(d_roof);
          thetai = theta;
          thetaa = thetai;
          thetaaa = thetaa;
          d_roofa = d_roof;
          ini = false;

          if (debug)
          {
            std::cout << "Reference position: " << std::endl;
            std::cout << "xi = " << xi << ", yi = " << yi << ", thetai = " << thetai*CV_PI/180 << ", rhoi = " << rho << std::endl;
            std::cout << "-----------------------------------------------------------" << std::endl;
          }
        }
      }
    }
  }
}

bool node_init(mission_manager::mission_manager::Request  &req, mission_manager::mission_manager::Response &res)
{
  std::cout << "[LIDAR]: service received" << std::endl;
  /*XG = req.x_pos;
  YG = req.y_pos;
  ZG = req.z_pos;
  YAWG = req.yaw_rot;
  takeoff = req.takeoff;*/
  std::cerr << "#############[DEBUG]: " << XG << " , " << YG << "#############" << std::endl;
  return true;
}

int main (int argc, char **argv)
{
  ROS_INFO("Starting Localization node with lidar info...");
  std::string _node_name = "lidar_localization";
  ros::init(argc, argv, _node_name);
  ros::NodeHandle n;

	//Filter parameters
	n.getParam(_node_name + "/ANGLE_MAX",max);	// Angulo maximo para filtrado
	n.getParam(_node_name + "/ANGLE_MIN",min);	// Angulo minimo para filtrado
	n.getParam(_node_name + "/MAX_DISTANCE",distance);  // Distancia maxima aceptada
  n.getParam(_node_name + "/DIS2LINE",d);  // Distancia maxima aceptada
  n.getParam(_node_name + "/OFFSET",ofs);
  n.getParam(_node_name + "/MIN_POINTS_LINE",min_points);
	n.getParam(_node_name + "/SCAN_TOPIC",scan_topic);
  n.getParam(_node_name + "/LASER_FRAME_ID",laser_frame_id);
  n.getParam(_node_name + "/ROT_XY",XY_hough);
  n.getParam(_node_name + "/ROT_XZ",XZ_hough);
  n.getParam(_node_name + "/PATH_FRAME_ID",path_frame_id);
  n.getParam(_node_name + "/ACCEPTED_ERROR",error);
  n.getParam(_node_name + "/DEBUG",debug); // Debug mode
  n.getParam(_node_name + "/FILTERED_LASER_TOPIC",filtered_laser_topic); 
  n.getParam(_node_name + "/FILTERED_MIRROR_TOPIC",filtered_mirror_topic); 
  n.getParam(_node_name + "/PATCH_TOPIC",path_topic); 
  n.getParam(_node_name + "/POSE_TOPIC",pose_topic); 
  n.getParam(_node_name + "/Z_TOPIC",z_topic); 
  n.getParam(_node_name + "/MISSION_MANAGER_SERVICE",mission_manager_service); 

  //Publishers
  pub2d = n.advertise<sensor_msgs::PointCloud2>("/laser2d", 100);
  pub = n.advertise<sensor_msgs::PointCloud>(filtered_laser_topic, 100);
  pub2 = n.advertise<sensor_msgs::PointCloud>(filtered_mirror_topic, 100);
  path = n.advertise<nav_msgs::Path>(path_topic, 100);
  high = n.advertise<std_msgs::Float32>(z_topic, 100);
  houghp = n.advertise<nav_msgs::Odometry>(pose_topic, 100);

  //Subscribers
  ros::Subscriber laser = n.subscribe(scan_topic, 100, lidar_callback);

  //Services
  ros::ServiceServer service = n.advertiseService(mission_manager_service, node_init);

  msg_path.header.frame_id = path_frame_id; 

  ROS_INFO("Waiting data...");

  //waiting messages
  ros::spin();

  return 0;
}
