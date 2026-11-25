/**
*   @file       DirectHough.cpp
*
*   @author     Juan Antonio Sanchez Diaz <juanantoniodiaz99@gmail.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Main node of laser localization using hough transform in the viaduct scenarios
**/

#include "laser.h"

// TODO: TENER EN CUENTA UN METODO DE LOCALIZACION EN FUNCION DE LA 
// DISTANCIA AL TECHO

/**
* @brief Constructor of the LaserGeneric class
**/
LaserGeneric::LaserGeneric(ros::NodeHandle &_node, std::string node_name)
{
  /* this se utiliza para acceder a los miembros de la instancia actual de la clase, 
  y se utiliza para distinguirlos de variables con el mismo nombre que puedan 
  estar en un ámbito más amplio.*/
  this->node_ = _node; // ros node handle
  this->name = node_name;

  // Get parameters
  getParameters();

  //Publishers
  pub2d = node_.advertise<sensor_msgs::PointCloud2>("/laser2d", 100);
  clus = node_.advertise<sensor_msgs::PointCloud2>(cluster_topic, 100);  
  pub = node_.advertise<sensor_msgs::PointCloud>(filtered_laser_topic, 100);
  pub2 = node_.advertise<sensor_msgs::PointCloud>(filtered_mirror_topic, 100);
  path = node_.advertise<nav_msgs::Path>(path_topic, 100);
  //high = node_.advertise<geometry_msgs::PoseStamped>(z_topic, 100);
  high = node_.advertise<std_msgs::Float32>(z_topic, 100);
  posepub = node_.advertise<nav_msgs::Odometry>(pose_topic, 100);

  /* se utiliza this para pasar un puntero a la instancia actual de la clase al método de 
  callback del suscriptor, de manera que éste pueda acceder a los miembros de la instancia actual. 
  Como self en python */
  //Subscribers
  lidar_sub = node_.subscribe(scan_topic, 1, &LaserGeneric::lidar_callback, this);
  
  //Services
  srv_manager = node_.advertiseService(mission_manager_service, &LaserGeneric::node_init, this);
  srv_alarm = node_.serviceClient<mission_manager::Alarms>(alarm_service);

  // Update frame id after get parameters 
  msg_path.header.frame_id = path_frame_id; 
}

/**
* @brief Destructor of the LaserGeneric class
**/
LaserGeneric::~LaserGeneric()
{
    ROS_INFO("[laser_generic]: Node has been killed");
}

void LaserGeneric::getParameters()
{
  std::string _node_name = this->name;

	node_.getParam(_node_name + "/ANGLE_MAX",i_max);	
	node_.getParam(_node_name + "/ANGLE_MIN",i_min);	
	node_.getParam(_node_name + "/MAX_DISTANCE",max_dis);  
  node_.getParam(_node_name + "/DIS2LINE",d); 
  node_.getParam(_node_name + "/OFFSET",ofs);
  node_.getParam(_node_name + "/MIN_POINTS_LINE",min_points);
  node_.getParam(_node_name + "/ROT_XY",XY_hough);
  node_.getParam(_node_name + "/ROT_XZ",XZ_hough);
  node_.getParam(_node_name + "/ACCEPTED_ERROR",error);
  node_.getParam(_node_name + "/DEBUG",debug); 
  node_.getParam(_node_name + "/MISSION_MANAGER_SERVICE",mission_manager_service); 
  node_.getParam(_node_name + "/ALARM_SERVICE",alarm_service); 
  node_.getParam(_node_name + "/EPSILON",epsilon); 
  node_.getParam(_node_name + "/MINIMUM_POINTS",minimum_points); 
	node_.getParam(_node_name + "/SCAN_TOPIC",scan_topic);
  node_.getParam(_node_name + "/FILTERED_LASER_TOPIC",filtered_laser_topic); 
  node_.getParam(_node_name + "/FILTERED_MIRROR_TOPIC",filtered_mirror_topic); 
  node_.getParam(_node_name + "/PATCH_TOPIC",path_topic); 
  node_.getParam(_node_name + "/CLUSTER_TOPIC",cluster_topic); 
  node_.getParam(_node_name + "/POSE_TOPIC",pose_topic); 
  node_.getParam(_node_name + "/Z_TOPIC",z_topic); 
  node_.getParam(_node_name + "/LASER_FRAME_ID",laser_frame_id);
  node_.getParam(_node_name + "/PATH_FRAME_ID",path_frame_id);

  // Print parameters
    ROS_INFO("\n Parameters: \n"
             "      i_max = %d \n"
             "      i_min = %d \n"
             "      d = %f \n"
             "      ofs = %d \n"
             "      min_points = %d \n"
             "      XY_hough = %f \n"
             "      XZ_hough = %f \n"
             "      error = %f \n"
             "      debug = %d \n"
             "      mission_manager_service = %s \n"
             "      alarm_service = %s \n"
             "      epsilon = %f \n"
             "      minimum_points = %f \n"
             "      scan_topic = %s \n"
             "      filtered_laser_topic = %s \n"
             "      filtered_mirror_topic = %s \n"
             "      path_topic = %s \n"
             "      cluster_topic = %s \n"
             "      pose_topic = %s \n"             
             "      z_topic = %s \n"
             "      laser_frame_id = %s \n"
             "      path_frame_id = %s \n",
             i_max, i_min, d, ofs, min_points,
             XY_hough, XZ_hough, error, debug, mission_manager_service.c_str(), 
             alarm_service.c_str(), epsilon, minimum_points, scan_topic.c_str(), filtered_laser_topic.c_str(), 
             filtered_mirror_topic.c_str(), path_topic.c_str(), cluster_topic.c_str(), 
             pose_topic.c_str(), z_topic.c_str(), laser_frame_id.c_str(), path_frame_id.c_str());
}

/**
* @brief Gets a vector with unique values from an input vector
**/ 
std::vector<int> LaserGeneric::unicos(std::vector<Point> cloud) 
{
    std::vector<int> vectorUnico;
    for (int i = 0; i < cloud.size(); i++) 
    {
      bool encontrado = false;
      for (int j = 0; j < vectorUnico.size(); j++) 
      {
        if (cloud[i].clusterID == vectorUnico[j]) 
        {
          encontrado = true;
          break;
        }
      }
      if (!encontrado) vectorUnico.push_back(cloud[i].clusterID);
    }

    return vectorUnico;
}

/**
* @brief Function for estimating the orientation of a defined cluster
**/ 
double LaserGeneric::angle_estimation(std::vector<Point> cloud, int cluster, vector<double> &centroid, pcl::PointCloud<pcl::PointXYZI> &clouds_pcl)
{
  vector<vector<double>> points(2, vector<double>(2));
  double dist = 0.0;
  for (int i = 0; i < cloud.size(); i++){
    if(cloud[i].clusterID == cluster){
      double norma = sqrt(pow(centroid[0]-cloud[i].x, 2) + pow(centroid[1]-cloud[i].y, 2));
      if (norma > dist){
        dist = norma;
        points[0][0] = cloud[i].x;
        points[0][1] = cloud[i].y;
      }
    }
  }
  dist = 0.0;
  for (int i = 0; i < cloud.size(); i++){
    if(cloud[i].clusterID == cluster){
      double norma = sqrt(pow(points[0][0]-cloud[i].x, 2) + pow(points[0][1]-cloud[i].y, 2));
      if (norma > dist){
        dist = norma;
        points[1][0] = cloud[i].x;
        points[1][1] = cloud[i].y;
      }
    }
  }

  if(debug)
  {
    std::cout << "Punto1 => " << "x: " << points[0][0] << " y: " << points[0][1] << std::endl;
    std::cout << "Punto2 => " << "x: " << points[1][0] << " y: " << points[1][1] << std::endl;
  }

  // Actualizamos el centroid para que no dependa de la densidad de la nube si no de la geometría
  centroid[0] = (points[0][0] + points[1][0]) / 2;
  centroid[1] = (points[0][1] + points[1][1]) / 2;

  pcl::PointXYZI data;
  for(auto point:points) 
  {
    data.x = point[0];
    data.y = point[1];
    data.z = 0;
    data.intensity = 0;
    clouds_pcl.push_back(data);
  }

  std::sort(points.begin(), points.end(), [](const std::vector<double>& a, const std::vector<double>& b) {
        return a[1] > b[1];
    });

  // Estimation of the object orientation
  vector<double> V1 {(points[0][0]-points[1][0]) / sqrt(pow(points[0][0]-points[1][0], 2) + pow(points[0][1]-points[1][1], 2))};
  V1.push_back((points[0][1]-points[1][1]) / sqrt(pow(points[0][0]-points[1][0], 2) + pow(points[0][1]-points[1][1], 2)));
  double angle = atan2(V1[1], V1[0]);

  return angle;
}

bool LaserGeneric::node_init(mission_manager::mission_manager::Request  &req, mission_manager::mission_manager::Response &res){
  std::cout << "[LIDAR]: service received" << std::endl;
  /*XG = req.x_pos;
  YG = req.y_pos;
  ZG = req.z_pos;
  YAWG = req.yaw_rot;
  takeoff = req.takeoff;*/
  std::cerr << "#############[DEBUG]: " << XG << " , " << YG << "#############" << std::endl;
  return true;
}

void LaserGeneric::lidar_callback(const  sensor_msgs::LaserScan::ConstPtr& msg){
  HealthCount = 0;
    if(Alarm_FLAG)
    {
        mission_manager::Alarms srv;
        srv.request.type = mission_manager::Alarms::Request::SensorsHealth; // TODO: CAMBIAR EL TIPO DE ALARMA
        srv.request.state = false; // ¿?
        srv.request.msg = "LiDAR camera working";
        srv_alarm.call(srv);

        Alarm_FLAG = false;
        Alarm_TYPE = mission_manager::Alarms::Request::SensorsHealth;
    }

  sensor_msgs::PointCloud data;
  sensor_msgs::PointCloud data_mirror;

  ros::WallTime start_, end_;
  start_ = ros::WallTime::now();

  sensor_msgs::LaserScan msg_laser = *msg;
  sensor_msgs::LaserScan mirror_laser = *msg;
  
  size_t longitud = msg->ranges.size(); 
 
  // ###################### ALARMAS #############################
  bool laserHealth = false;
  for(const auto& range : msg_laser.ranges){
      if(range != 0 && !std::isinf(range)){
          laserHealth = true; // Si encontramos un valor distinto de 0 o infinito, establecemos la bandera en falso.
          break; 
      }
  }
  if(!laserHealth){
    mission_manager::Alarms srv;
    srv.request.type = mission_manager::Alarms::Request::SensorsHealth;
    srv.request.state = true;
    srv.request.msg = "LiDAR not working";
    srv_alarm.call(srv);

    Alarm_FLAG = true;
    Alarm_TYPE = mission_manager::Alarms::Request::SensorsHealth;
  }

  bool Obs_detected = false;
  for(const auto& range : msg_laser.ranges){
      if(range <= 1){ // 1 metro de distancia
          Obs_detected = true; // Si encontramos un valor cercano
          break; 
      }
  }
  if(!Obs_detected){
    mission_manager::Alarms srv;
    srv.request.type = mission_manager::Alarms::Request::Obstacle;
    srv.request.state = true;
    srv.request.msg = "Obstacle detected with LiDAR";
    srv_alarm.call(srv);

    Alarm_FLAG = true;
    Alarm_TYPE = mission_manager::Alarms::Request::Obstacle;
  }
  // #########################################################


  i_max = round(longitud/2) + ofs; // 433
  i_min = round(longitud/2) - ofs; // 292
  float zaux = 0;
  float total = 0;
  for(int k = 0; k < msg_laser.ranges.size(); k++){
    if(msg_laser.ranges[k] == std::numeric_limits<float>::infinity()){
      msg_laser.ranges[k] = 0.0;
    }
    if(k > i_min && k < i_max){
        if(msg_laser.ranges[k] != 0.0 && msg_laser.ranges[k] <= 10000){
          zaux += msg_laser.ranges[k];
          total++;
        }else{
          mirror_laser.ranges[k] = 0.0;
        }
        msg_laser.ranges[k] = 0.0;
    }else{
      mirror_laser.ranges[k] = 0.0;
    }
    if(msg_laser.ranges[k] >= max_dis || (k < i_max + ofs + 100 && k > i_min - ofs - 100)){ // || msg_laser.ranges[k] <= 1
      msg_laser.ranges[k] = 0.0;
    }
  }

  zaux = zaux / total;

  msg_laser.header.frame_id = laser_frame_id;

  // Updating the new msgs
  projector_.projectLaser(msg_laser, data);
  data.header.frame_id = laser_frame_id;
  pub.publish(data);

  projector_.projectLaser(mirror_laser, data_mirror);
  pub2.publish(data_mirror);

  double theta = 0;
  float d_roof = 0;
  pcl::PointCloud<pcl::PointXYZI> cloud_cluster;
  pcl::PointXYZI point;
  std::vector<int> clusters;
  int ir_max = 0;
  int it_max = 0;
  int i_max = 0;
  int escala;
  if(true){ // ceil_distance.data > 0.05
    //######################## MIRROR HOUGH TRANSFORM ###############################
    escala = 100; // Rho resolution
    int dist = max_dis; // Max distance of the lines
    int i_R = dist*escala;
    int i_T = 360;
    
    _accu = (unsigned int*)calloc(i_R * i_T, sizeof(unsigned int));

    double center_x = 0;
    double center_y = 0;

    // Rellenamos matriz de votos de Hough
    for(int i = 0; i<data_mirror.points.size(); i++){
      float x = data_mirror.points[i].y;
      float y = data_mirror.points[i].x;
      for(int t=0;t<i_T;t++){
        float r = ((x - center_x) * cos((double)t * PI/180)) + (((double)y - center_y) * sin((double)t * PI/180));
        r = abs(round(r*escala));
        _accu[(int)((r * i_T)) + t]++;
      }
    }
    
    if(!ini){ // Buscar cerca del punto anterior
      int ran_min = ir_max_a-30; 
      if (ran_min < 0) ran_min = 0; // limitar busqueda a 0
      for(int r=ran_min;r<ir_max_a+30;r++){
        for(int t=90-20;t<90+20;t++){ // la linea se encuentra centrada en 90 // t=it_max_a-10;t<it_max_a+10
          if(it_max_a-10 >= 0){
            if(_accu[(int)((r * i_T)) + t] > i_max){
              i_max = _accu[(int)((r * i_T)) + t];
              ir_max = r;
              it_max = t;
            }
          } 
        }
      }
      if (abs(it_max_a - it_max) < 20) it_max_a = it_max;
      //else {it_max = it_max_a;}
      if (abs(ir_max_a - ir_max) < 20) ir_max_a = ir_max;
      //else {ir_max = ir_max_a;}
    }else{ 
      for(int r=0;r<i_R;r++){
        for(int t=0;t<i_T;t++){
          if(_accu[(int)((r * i_T)) + t] > i_max){ 
            i_max = _accu[(int)((r * i_T)) + t];
            ir_max = r;
            it_max = t;
          }
        }
      }
      it_max_a = it_max;
      ir_max_a = ir_max;
    }

    d_roof = abs((ir_max - i_R)/(float)escala);
    
    if(!ini){
      if(abs(d_roof - d_roofa) < 0.5){
        d_roofa = d_roof;
      }else{
        d_roof = d_roofa;
      }
    }

    //######################## DBSCAN ###############################
    std::vector<Point> points;
    Point p;

    for(int i = 0; i < data.points.size(); i++){
      p.x = data.points[i].x;
      p.y = data.points[i].y;
      p.z = 0.0;
      p.clusterID = UNCLASSIFIED;
      points.push_back(p);
      // printf("%f, %f, %f\n", (points[i].x), (points[i].y), (points[i].z));
    }

    DBSCAN *ds;
    if(ceil_distance.data > 0.1){
      ds = new DBSCAN(minimum_points, epsilon, points);
      if(debug) std::cout << "DBSCAN => " << "EPSILON: " << epsilon << " MINIMUM_POINTS: " << minimum_points << std::endl;
    }else{
      float modify = 1;
      ds = new DBSCAN(minimum_points*modify, epsilon*modify, points);
      if(debug) std::cout << "DBSCAN => " << "EPSILON: " << epsilon*modify << " MINIMUM_POINTS: " << minimum_points*modify << std::endl;
    }

    ds->run();

    points = ds->m_points;
    clusters = unicos(points); // Classes Vector


///*
// In this block, only one pillar is detected and if the pillar is lost, there is no new pillar
// to be taken as the reference

    int grupoDeseado = 0;
    float xCM = 0, yCM = 0;        
    float sumx = 0, sumy = 0;
    float n_points = 0;
    for (int cluster:clusters){
      sumx = 0; sumy = 0; n_points = 0;
      for (int i = 0; i < data.points.size(); i++){
        if (points[i].clusterID == cluster){
          sumx += points[i].x;
          sumy += points[i].y;
          n_points++;
        }
      } 
      
      xCM = sumx/n_points; 
      yCM = sumy/n_points;
      
      // TODO: AÑADIR CONDICION DE BUSQUEDA DE PILAR
      // - PRIMERO COMPARAR CON EL ID ANTERIOR, SI NO COINCIDE, BUSQUEDA
      // GLOBAL Y CAMBIAMOS ID ANTERIOR

      //  
      if (!ini){
        if(debug){
          std::cout << abs(xCMa - xCM) + abs(yCMa - yCM) << std::endl;
          std::cout << abs(int(points.size() - size_a)) << std::endl;
          std::cout << cluster << std::endl;
        }
        if((abs(xCMa - xCM) + abs(yCMa - yCM) < 1.5) && int(points.size()) > 20 && cluster != -1){
          size_a = points.size();
          xCMa = xCM; 
          yCMa = yCM;
          grupoDeseado = cluster;
          break;
        }
      }else{
        if(cluster == 1){
          size_a = points.size();
          xCMa = xCM; 
          yCMa = yCM;
          grupoDeseado = cluster;
          break;
        }
      }
    }

//*/

/*
// With this block the LiDAR always search for new pillars
// ################# IMPROVED PILLAR TRACKING #################
    int grupoDeseado = -1;
    float xCM = 0, yCM = 0;        
    float sumx = 0, sumy = 0;
    float n_points = 0;
    bool match_found = false;

    // 1. FIRST PASS: Try to find the OLD pillar (Continuity Check)
    for (int cluster : clusters) {
        if (cluster == -1) continue; // Skip noise

        // Calculate Center of Mass for this cluster
        sumx = 0; sumy = 0; n_points = 0;
        for (int i = 0; i < data.points.size(); i++) {
            if (points[i].clusterID == cluster) {
                sumx += points[i].x;
                sumy += points[i].y;
                n_points++;
            }
        }
        if (n_points == 0) continue;
        float current_xCM = sumx / n_points;
        float current_yCM = sumy / n_points;

        // If we are initializing, just take the first valid cluster (or specific ID 1)
        if (ini) {
            if (cluster == 1 || clusters.size() == 1) { // Take cluster 1 or if it's the only one
                xCM = current_xCM; yCM = current_yCM;
                grupoDeseado = cluster;
                match_found = true;
                break;
            }
        } 
        // If we are already flying, look for the PREVIOUS pillar
        else {
            if ((abs(xCMa - current_xCM) + abs(yCMa - current_yCM) < 1.5) && n_points > 20) {
                xCMa = current_xCM; yCMa = current_yCM;
                size_a = n_points;
                xCM = current_xCM; yCM = current_yCM;
                grupoDeseado = cluster;
                match_found = true;
                break;
            }
        }
    }

    // 2. HANDOVER LOGIC: If Old Pillar is LOST, find a NEW one
    if (!ini && !match_found) {
        if(debug) std::cout << "Old pillar lost! Searching for new reference..." << std::endl;
        
        float best_dist = 1000.0;
        int best_cluster = -1;
        float best_xCM = 0, best_yCM = 0;

        // Loop again to find the CLOSEST valid cluster to the drone (or the biggest)
        for (int cluster : clusters) {
            if (cluster == -1) continue;
            
            sumx = 0; sumy = 0; n_points = 0;
            for (int i = 0; i < data.points.size(); i++) {
                if (points[i].clusterID == cluster) {
                    sumx += points[i].x;
                    sumy += points[i].y;
                    n_points++;
                }
            }
            
            if (n_points < 20) continue; // Must be a solid pillar

            float current_xCM = sumx / n_points;
            float current_yCM = sumy / n_points;

            // Simple heuristic: Pick the cluster closest to the LiDAR center (0,0)
            float dist_to_drone = sqrt(pow(current_xCM, 2) + pow(current_yCM, 2));
            
            if (dist_to_drone < best_dist) {
                best_dist = dist_to_drone;
                best_cluster = cluster;
                best_xCM = current_xCM;
                best_yCM = current_yCM;
            }
        }

        if (best_cluster != -1) {
            if(debug) std::cout << "Pillar Switch! New Cluster ID: " << best_cluster << std::endl;
            
            // 1. Set the Desired Cluster to this new one
            grupoDeseado = best_cluster;
            xCM = best_xCM; yCM = best_yCM;
            xCMa = best_xCM; yCMa = best_yCM; // Update tracking variable
            
            // 2. CALCULATE NEW REFERENCE 
            pcl::PointCloud<pcl::PointXYZI> temp_cloud;
            vector<double> temp_centroid = {xCM, yCM};
            double new_theta = angle_estimation(points, grupoDeseado, temp_centroid, temp_cloud);
            
            // Update Orientation Reference
            double current_yaw = -(thetaa - thetai); 
            thetai = new_theta + current_yaw; 
            
            // Update Position Reference            
            xi = (xCM * cos(current_yaw) - yCM * sin(current_yaw)) - xa;
            yi = (xCM * sin(current_yaw) + yCM * cos(current_yaw)) - ya;

            // if(debug) std::cout << "Reference Updated." << std::endl;
            match_found = true;
        } else {
            // ROS_ERROR("CRITICAL: No pillars visible! Localization lost.");
        }
    }
    // ############################################################
*/

    for (int i = 0; i < data.points.size(); i++){
      if(points[i].clusterID == grupoDeseado){
        point.x = points[i].x;
        point.y = points[i].y;
        point.z = 0;
        point.intensity = 0;
        cloud_cluster.push_back(point);
      }
    }

    sensor_msgs::PointCloud2 output_ros2;
    pcl::toROSMsg(cloud_cluster, output_ros2);
    output_ros2.header = msg_laser.header;
    clus.publish(output_ros2);

    pcl::PointCloud<pcl::PointXYZI> clouds1_pcl;

    // Estimate the angle of the object
    vector<double> centroid = {xCM, yCM};
    theta = angle_estimation(points, grupoDeseado, centroid, clouds1_pcl);

    point.x = centroid[0];
    point.y = centroid[1];
    point.z = 0;
    point.intensity = 0;
    clouds1_pcl.push_back(point);

    if(debug){
      std::cout << "xCM: " << xCM << " yCM: " << yCM << std::endl;
      std::cout << "angle: " << theta << std::endl;
    }

    sensor_msgs::PointCloud2 output_ros;
    pcl::toROSMsg(clouds1_pcl, output_ros);
    output_ros.header = msg_laser.header;
    pub2d.publish(output_ros);
  }else{
    d_roof = d_roofa; // Mantenemos altura

    // TODO: LOCALIZACION BASADA EN ZONA PEGADA AL TECHO


    // #################################################
  }

  if (!ini){
    // if(thetai < 0 && theta > 0) theta = theta - 3.1416;
    // else if(thetai > 0 && theta < 0) theta = theta + 3.1416;

    theta = (theta + thetaa) / 2;
    yaw = -(theta - thetai);
    
    m.pose.position.x = (point.x*cos(yaw) - point.y*sin(yaw)) - xi; 
    m.pose.position.y = (point.x*sin(yaw) + point.y*cos(yaw)) - yi;
    m.pose.position.z = d_roof - zi; //sqrt(0.05*abs(m.pose.position.x)); //0.087*abs(m.pose.position.x); // sqrt(0.15*abs(m.pose.position.y))

    //ROS_INFO("[LIDAR TEST] Pillar position relative to start (X, Y): %f, %f", m.pose.position.x, m.pose.position.y);

    // end_ = ros::WallTime::now();
    // double execution_time = (end_ - start_).toNSec() * 1e-6;

    // Print results
    if(debug){
      std::cout << "Relative position: " << std::endl;
      std::cout << "x = " << point.x << ", y = " << point.y << ", theta = " << theta*180/PI << std::endl;
      std::cout << "xt = " << (xi*cos(yaw) - yi*sin(yaw)) << ", yt = " << (xi*sin(yaw) + yi*cos(yaw)) << std::endl;
      std::cout << "Xg = " << m.pose.position.x << ", Yg = " << m.pose.position.y << ", yawg = " << yaw*180/PI << std::endl;
      std::cout << "Número de clusters = " << clusters.size() << std::endl;
      //std::cout << "Execution time: " << execution_time << std::endl;
      std::cout << "-----------------------------------------------------------" << std::endl;
    }
    
    m.pose.position.x = m.pose.position.x + XG;
    m.pose.position.y = m.pose.position.y + YG;
    m.pose.position.z = m.pose.position.z + ZG;

    // Smoothing
    if (cont_med >= 3){
      m.pose.position.x = (m.pose.position.x + xa + xaa) / 3;
      m.pose.position.y = (m.pose.position.y + ya + yaa) / 3;
      m.pose.position.z = (m.pose.position.z + za) / 2;
    } 

    if(abs(m.pose.position.x - xa) < error && abs(m.pose.position.y - ya) < error && abs(m.pose.position.z - za) < error){
      if (cont_med < 3) cont_med++;

      // Save old information
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

      tf::Quaternion q_t = tf::createQuaternionFromRPY(0, 0, yaw*PI/180 + YAWG); //- XY_hough*PI/180
      geometry_msgs::Quaternion _q_pose;
      _q_pose.x = q_t.x();
      _q_pose.y = q_t.y();
      _q_pose.z = q_t.z();
      _q_pose.w = q_t.w();
      m.pose.orientation = _q_pose;

      //std::cout << _q_pose << std::endl; 

      msg_path.header.stamp = ros::Time::now(); // <--- ADD THIS
      msg_path.poses.push_back(m);
      path.publish(msg_path);
      msg_path.header.seq++;
      
      nav_msgs::Odometry odom;
      odom.pose.pose = m.pose;
      odom.header.frame_id = path_frame_id;
      odom.header.stamp = ros::Time::now();   // <--- ADD THIS EXACT LINE HERE
      posepub.publish(odom);
    }
  }else{
    // Reference point
    xi = point.x;
    yi = point.y;
    zi = abs(d_roof);
    thetai = theta;

    // Smoothing initialization
    thetaa = thetai;
    thetaaa = thetaa;
    d_roofa = d_roof;
    ini = false;

    if (debug){
      std::cout << "Reference position: " << std::endl;
      std::cout << "xi = " << xi << ", yi = " << yi << ", thetai = " << thetai*180/PI << std::endl;
      std::cout << "-----------------------------------------------------------" << std::endl;
    }
  }

  //mz.pose.position.z = d_roof - zi;
  ceil_distance.data = ir_max/(float)escala - 0.165;
  //ROS_INFO("[LIDAR TEST] Calculated distance to ceiling (Z): %f meters", ceil_distance.data);
  high.publish(ceil_distance);
}

/**
* @brief ROS spin loop
**/
void LaserGeneric::spin(){
  ros::spin();
}

void LaserGeneric::spinOnce(){
  ros::spinOnce();
  HealthCount++; // Wrapping
  if(HealthCount > 15){
      mission_manager::Alarms srv;
      srv.request.type = mission_manager::Alarms::Request::SensorsHealth; // TODO: CAMBIAR EL TIPO DE ALARMA
      srv.request.state = true; // ¿?
      srv.request.msg = "LiDAR not working";
      srv_alarm.call(srv);

      Alarm_FLAG = true;
      Alarm_TYPE = mission_manager::Alarms::Request::SensorsHealth;
  }
}