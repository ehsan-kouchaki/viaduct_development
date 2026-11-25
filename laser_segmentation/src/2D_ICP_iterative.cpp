#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>

laser_geometry::LaserProjection projector_; 
pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher previous_cloud_pub ,aligned_cloud_pub, transformed_cloud_pub;

bool init = true;

double x, y, z, yaw = 0.0;
double delta_x, delta_y, delta_z, delta_yaw = 0.0;

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    // ROS_INFO("LaserScan received.");

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
    for (int k = 0; k < hor_laser.ranges.size(); k++){
        if (k > min && k < max){
            if (hor_laser.ranges[k] == std::numeric_limits<float>::infinity() || hor_laser.ranges[k] > 10000){
                hor_laser.ranges[k] = 0.0;
                ver_laser.ranges[k] = 0.0;
            }
        }else{
            ver_laser.ranges[k] = 0.0;
        }
        if(hor_laser.ranges[k] >= distance || (k < max + offset + 100 && k > min - offset - 100) || hor_laser.ranges[k] <= 1){
            hor_laser.ranges[k] = 0.0;
        }
    }

    sensor_msgs::PointCloud hor_data, ver_data, final_scan;
    projector_.projectLaser(hor_laser, hor_data);
    projector_.projectLaser(ver_laser, ver_data);

    for(size_t i = 0; i < hor_data.points.size(); ++i) final_scan.points.push_back(hor_data.points[i]);

    for(size_t i = 0; i < ver_data.points.size(); ++i){
        // Obtener el punto actual
        geometry_msgs::Point32 point = ver_data.points[i];

        // Intercambiar los valores de x por z
        point.z = point.x;
        point.x = 0;

        // Agregar el punto transformado a la nube de puntos personalizada
        // final_scan.points.push_back(point);
    }

    // Crear la nube de puntos transformada
    sensor_msgs::PointCloud transformedCloud;
    transformedCloud.header = final_scan.header;

    // Transformar cada punto en la nube de puntos original
    for (size_t i = 0; i < final_scan.points.size(); ++i)
    {
        // Obtener el punto actual
        geometry_msgs::Point32 point = final_scan.points[i];
        geometry_msgs::Point32 pointf = final_scan.points[i];

        // Actualizar las coordenadas del punto transformado 
        pointf.x = point.x - (delta_x*cos(yaw) - delta_y*sin(yaw));
        pointf.y = point.y - (delta_x*sin(yaw) + delta_y*cos(yaw));
        pointf.z = point.z + delta_z;

        // Agregar el punto transformado a la nube de puntos transformada
        transformedCloud.points.push_back(pointf);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloudPCL(new pcl::PointCloud<pcl::PointXYZ>);
    transformedCloudPCL->points.resize(transformedCloud.points.size());

    for (size_t i = 0; i < transformedCloud.points.size(); ++i)
    {
        transformedCloudPCL->points[i].x = transformedCloud.points[i].x;
        transformedCloudPCL->points[i].y = transformedCloud.points[i].y;
        transformedCloudPCL->points[i].z = transformedCloud.points[i].z;
    }

    transformedCloud.header.frame_id = "laser";  // Reemplaza "map_frame" por el marco de referencia correcto
    transformedCloud.header.stamp = ros::Time::now();
    transformed_cloud_pub.publish(transformedCloud);

    if(init){
        previous_cloud = transformedCloudPCL; //crea un objeto compartido (es un puntero)
        init = false;
    }
    else{
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(transformedCloudPCL);
        icp.setInputTarget(previous_cloud);

        pcl::PointCloud<pcl::PointXYZ> final_output;
        icp.align(final_output);

        if (icp.hasConverged())
        {
            // ROS_INFO("ICP Converged!");
            // ROS_INFO("Transformación estimada:\n");
            // std::cout << icp.getFinalTransformation().matrix() << std::endl;
        }

        Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();

        // delta_x = transformation_matrix(0, 3); // Componente X de la translación
        // delta_y = transformation_matrix(1, 3); // Componente Y de la translación
        // delta_z = transformation_matrix(2, 3); // Componente Z de la translación

        sensor_msgs::PointCloud2 previous_cloud_msg;
        pcl::toROSMsg(*previous_cloud, previous_cloud_msg);
        previous_cloud_msg.header.frame_id = "laser";  // Reemplaza "map_frame" por el marco de referencia correcto
        previous_cloud_msg.header.stamp = ros::Time::now();
        previous_cloud_pub.publish(previous_cloud_msg);

        std::cout << "icpX = " << transformation_matrix(0, 3) << ", icpY = " << transformation_matrix(1, 3) << std::endl;
        std::cout << "vodX = " << delta_x << ", vodY = " << delta_y << std::endl;
        std::cout << sqrt(pow(transformation_matrix(0, 3) - delta_x,2) + pow(transformation_matrix(1, 3) - delta_y,2)) << std::endl;
        // std::cout << final_output.points.size() << std::endl;
        if(true){ // add condition to detect if exit a wrong match
            previous_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(final_output); //crea un objeto compartido (es un puntero)
        }
        // std::cout << previous_cloud->points.size() << std::endl;
        
        sensor_msgs::PointCloud2 aligned_cloud_msg;
        pcl::toROSMsg(final_output, aligned_cloud_msg);
        aligned_cloud_msg.header.frame_id = "laser";  // Reemplaza "map_frame" por el marco de referencia correcto
        aligned_cloud_msg.header.stamp = ros::Time::now();
        aligned_cloud_pub.publish(aligned_cloud_msg);
    }

}   

void PoseCallback(const  nav_msgs::Odometry::ConstPtr& msg){ //geometry_msgs::PoseStamped nav_msgs::Odometry
//   ROS_INFO("PoseStamped received.");

  // Supongamos que recibes las coordenadas X, Y y Z a sumar
  x = msg->pose.pose.position.x; //pose.
  y = msg->pose.pose.position.y;
  z = msg->pose.pose.position.z;

  // Supongamos que recibes el ángulo YAW en radianes para la rotación
  tf::Quaternion quaternion(msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z,
                            msg->pose.pose.orientation.w); // (x, y, z, w)

  // Convertir el cuaternión a roll, pitch y yaw
  yaw = tf::getYaw(quaternion) - 3.1416/2; //

//   std::cout << "Pose => " << "X = " << x << " Y = " << y  
//   << " Z = " << z << " yaw = " << yaw << std::endl;
}

void OdomCallback(const  geometry_msgs::PoseStamped::ConstPtr& msg){ //geometry_msgs::PoseStamped nav_msgs::Odometry
//   ROS_INFO("OdomStamped received.");

  // Supongamos que recibes las coordenadas X, Y y Z a sumar
  delta_x = msg->pose.position.x; //pose.
  delta_y = msg->pose.position.y;
  delta_z = msg->pose.position.z;

  // Supongamos que recibes el ángulo YAW en radianes para la rotación
  tf::Quaternion quaternion(msg->pose.orientation.x,
                            msg->pose.orientation.y,
                            msg->pose.orientation.z,
                            msg->pose.orientation.w); // (x, y, z, w)

  // Convertir el cuaternión a roll, pitch y yaw
  delta_yaw = tf::getYaw(quaternion) - 3.1416/2; //

//   std::cout << "Pose => " << "deltaX = " << delta_x << " deltaY = " << delta_y  
//   << " deltaZ = " << delta_z << " delta_yaw = " << delta_yaw << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_registration");
    ros::NodeHandle nh;

    // Suscribirse al tópico que proporciona la nube de puntos del sensor
    ros::Subscriber sensor_cloud_sub = nh.subscribe("/scan", 100, LaserCallback);
    ros::Subscriber pose = nh.subscribe("/track/pose", 100, PoseCallback); // /mavros/vision_pose /totalstation
    ros::Subscriber odom = nh.subscribe("/track/odom", 100, OdomCallback); // /mavros/vision_pose /totalstation

    aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 1, true);
    previous_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("previous_cloud", 1, true);
    transformed_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("transformed_cloud", 1, true);

    ros::spin();

    return 0;
}