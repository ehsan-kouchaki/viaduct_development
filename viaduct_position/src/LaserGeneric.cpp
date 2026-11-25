/**
*   @file       DirectHough.cpp
*
*   @author     Juan Antonio Sanchez Diaz <juanantoniodiaz99@gmail.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Main node of laser localization using hough transform in the viaduct scenarios
*/

#include "laser.h"

LaserGeneric::LaserGeneric(ros::NodeHandle &_node, tf::TransformBroadcaster *tfbr_)
{
  this->node_ = _node; // ros node handle
  first_time_ = true;  // until first_time flag is false, the filter is not initialized
  //tag_found_ = false;       // initially no tag sensor has been found
  first_meas_ = true;        // until the first_meas is true, the local position is not published
  x_0_ = y_0_ = z_0_ = 0.0f; // initial estimated position of the drone
  takeoff_stage_ = true;
  br_ = tfbr_;
  lidar_received_ = false; // if first lidar measurement has been received

  //Publishers
  pub2d = n.advertise<sensor_msgs::PointCloud2>("/laser2d", 100);
  pub = n.advertise<sensor_msgs::PointCloud>("/laser_filtered", 100);
  pub2 = n.advertise<sensor_msgs::PointCloud>("/laser_filtered_mirror", 100);
  path = n.advertise<nav_msgs::Path>("/hough/path", 100);
  high = n.advertise<geometry_msgs::PoseStamped>("/hough/z", 100);
  houghp = n.advertise<nav_msgs::Odometry>("/hough/pose", 100);

  //Subscribers
  ros::Subscriber laser = n.subscribe(scan_topic, 1, lidar_callback);
  //ros::Subscriber total = n.subscribe("/totalstation/pose", 100, Position_pub);

  //ros::ServiceServer service = n.advertiseService("mission_manager_hough", node_init);

  msg_path.header.frame_id = "base_link_init"; //
}

/**
      * @brief Destructor of the KFLocalization class
*/
LaserGeneric::~LaserGeneric()
{
    ROS_INFO("[direct_hough]: Node has been killed");
}

void LaserGeneric::lidar_callback(const  sensor_msgs::LaserScan::ConstPtr& msg)
{
  //if(!takeoff)
  //{
    // MEDIR TIEMPO DE COMPUTO pcl o ros:time
    ros::WallTime start_, end_;
    start_ = ros::WallTime::now();

    sensor_msgs::LaserScan msg_laser = *msg;
    sensor_msgs::LaserScan mirror_laser = *msg;
    
    size_t longitud = msg->ranges.size(); //Vector's lenght

    int ofs = 100;
    max = round(longitud/2) + ofs; //433
    min = round(longitud/2) - ofs; //292
    float zaux = 0;
    float total = 0;
    for(int k = 0; k < msg_laser.ranges.size(); k++) //Estudio los puntos del mensaje y los distingo
    {
      if(msg_laser.ranges[k] == std::numeric_limits<float>::infinity()) msg_laser.ranges[k] = 0.0;

      // Mirror data
      if(k > min && k < max)
      {
        if(msg_laser.ranges[k] != 0.0 && msg_laser.ranges[k] <= 10000)  //msg_laser.ranges[k] < zaux && 
        {
          zaux += msg_laser.ranges[k];
          total++;
        }
        else
        {
          mirror_laser.ranges[k] = 0.0;
        }
        msg_laser.ranges[k] = 0.0;
      } 
      else
      {
        mirror_laser.ranges[k] = 0.0;
      }

      if(msg_laser.ranges[k] >= distance || (k < max + ofs + 100 && k > min - ofs - 100))
      {
        msg_laser.ranges[k] = 0.0;
      } 
    }

    zaux = zaux / total;

    msg_laser.header.frame_id = "laser";

    //Updating the new msgs
    projector_.projectLaser(msg_laser, data);
    data.header.frame_id = "laser";
    pub.publish(data);

    projector_.projectLaser(mirror_laser, data_mirror);
    pub2.publish(data_mirror);
    //std::cout << "debug3" << std::endl;

    //######################## MIRROR HOUGH TRANSFORM ###############################
    int escala = 100; //Resolucion en rho
    int dist = distance; //Distancia maxima de las rectas
    int i_R = dist*escala*2;
    int i_T = 180;
    
    _accu = (unsigned int*)calloc(i_R * i_T, sizeof(unsigned int));

    double center_x = 0;
    double center_y = 0;

    // Rellenamos matriz de votos de Hough
    for(int i = 0; i<data_mirror.points.size(); i++)
    {
      float x = data_mirror.points[i].x;
      float y = data_mirror.points[i].y;
      for(int t=0;t<i_T;t++)
      {
        double r = ( (x - center_x) * cos((double)t * CV_PI/180)) + (((double)y - center_y) * sin((double)t * CV_PI/180));
        r = round(r*escala) + i_R/2;
        _accu[(int)((r * i_T)) + t]++;
      }
    }

    int ir_max = 0;
    int it_max = 0;
    int max = 0;
    //std::cout << "debug2" << std::endl;
    
    /*for(int r=0;r<i_R;r++)
    {
      for(int t=0;t<i_T;t++)
      {
        if (_accu[(int)((r * i_T)) + t] > max)
        {
          max = _accu[(int)((r * i_T)) + t];
          ir_max = r;
          it_max = t;
        }
      }
    }*/

    if(!ini)
    {
      for(int r=ir_max_a2-25;r<ir_max_a2+25;r++)
      {
        for(int t=0;t<i_T;t++) // Cuidado con los giros
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
        for(int t=0;t<i_T;t++)
        {
          if (_accu[(int)((r * i_T)) + t] > 10) // Numero minimo de puntos para deteccion
          {
            max = _accu[(int)((r * i_T)) + t];
            ir_max = r;
            it_max = t;
          }
        }
      }
    }

    ir_max_a2 = ir_max;

    float d_roof = abs((ir_max - i_R/2)/(float)escala);

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
    escala = 100; //Resolucion en rho
    dist = distance; //Distancia maxima de las rectas
    i_R = dist*escala*2;
    i_T = 180;

    _accu = (unsigned int*)calloc(i_R * i_T, sizeof(unsigned int));

    // Rellenamos matriz de votos de Hough
    for(int i = 0; i<data.points.size(); i++)
    {
      float x = data.points[i].x;
      float y = data.points[i].y;
      for(int t=0;t<i_T;t++)
      {
        float r = ((x - center_x) * cos((double)t * CV_PI/180)) + (((double)y - center_y) * sin((double)t * CV_PI/180));
        r = round(r*escala) + i_R/2;
        _accu[(int)((r * i_T)) + t]++;
      }
    }
    
    ir_max = 0;
    it_max = 0;
    max = 0;
    //std::cout << ir_max_a << std::endl;
    if(!ini)
    {
      for(int r=ir_max_a-25;r<ir_max_a+25;r++)
      {
        for(int t=it_max_a-25;t<it_max_a+25;t++) // Cuidado con los giros
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
      for(int r=ir_max_a-500;r<ir_max_a+500;r++)
      {
        for(int t=0;t<i_T;t++)
        {
          if (_accu[(int)((r * i_T)) + t] > 10) // Numero minimo de puntos para deteccion
          {
            max = _accu[(int)((r * i_T)) + t];
            ir_max = r;
            it_max = t;
          }
        }
      }
    }

/*     for(int r=0;r<i_R;r++)
    {
      for(int t=0;t<i_T;t++)
      {
        if (_accu[(int)((r * i_T)) + t] > max)
        {
          max = _accu[(int)((r * i_T)) + t];
          ir_max = r;
          it_max = t;
        }
      }
    } */  

    if(max > 0) // Hay deteccion de linea
    {
      ir_max_a = ir_max;
      it_max_a = it_max;
      int theta = it_max;
      float rho = (ir_max - i_R/2)/(float)escala;

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
        m00 = 0, m01 = 0, m10 = 0;
        for (int i = 0; i < data.points.size(); i++)
        {
          float xi = data.points[i].x;
          float yi = data.points[i].y;
          float dCM = sqrt(pow((xi-xCM),2) + pow((yi-yCM),2));
          if (theta*CV_PI/180 != 0)
          {
            float yl = (rho - xi*cos(theta*CV_PI/180)) / sin(theta*CV_PI/180);
            if (abs((yi-yl)*sin(theta*CV_PI/180)) < d && dCM < 4)
            {
              m00 += 1;
              m10 += xi;
              m01 += yi;
            }
          }
          else
          {
            if (abs(xi - rho) < d && dCM < 4)
            {
              m00 += 1;
              m10 += xi;  
              m01 += yi;
            }
          }
        }

        xCM = m10/m00;
        yCM = m01/m00;

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
          if (rho < 0) theta = 180 + theta; 

          theta = (theta + thetaa) / 2;

          yaw = theta - thetai;
          
          m.pose.position.x = point.x - (xi*cos(yaw*CV_PI/180) - yi*sin(yaw*CV_PI/180)); 
          m.pose.position.y = point.y - (xi*sin(yaw*CV_PI/180) + yi*cos(yaw*CV_PI/180));
          m.pose.position.z = zi - d_roof;

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
            odom.header.frame_id = odom_frame_id;
            houghp.publish(odom);
          }
        }
        else
        {
          xi = point.x;
          yi = point.y;
          zi = abs(d_roof);
          if (rho < 0) theta = 180 + theta;
          thetai = theta;
          thetaa = thetai;
          thetaaa = thetaa;
          d_roofa = d_roof;
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
    mz.pose.position.z = zi - d_rof;
    high.publish(mz);
  //}
}

/**
      * @brief ROS spin loop
*/
void LaserGeneric::spin()
{
    ros::spin();
}