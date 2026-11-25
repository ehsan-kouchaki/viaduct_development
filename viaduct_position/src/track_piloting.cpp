/**
*   @file       homo_piloting.cpp
*
*   @author     Juan Antonio Sanchez Diaz <juanantoniodiaz99@gmail.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Main node of camera odometry in the viaduct scenarios using 
**/

#include "Pyramidal.h"

// TRACK PARAMETERS
int min_points = 20;
int new_points = 15;
int init_points = 100;
int lost_points = 5;
int advise_threshold = 7;
float edge_threshold = 0.2;
int nrows = 3, ncols = 4;
float minDistance = 30;
int blockSize = 7;
float responseK = 0.04;
float escala = 1.15; //0.68;

// TOPIC'S NAME
string image_topic = "/camera/color/image_raw";
string imageORB_topic = "/camera/ORB_image";
string altimeter_topic = "/sf11";
string path_topic = "/homo/path";
string pose_topic = "/homo/pose";
string odom_topic = "/homo/odom";
string mission_manager_service = "mission_manager_homo";

string odom_frame_id = "world";

bool debug = true;

float XG = 0, YG = 0, YAWG = 0;
bool takeoff = false;
Eigen::MatrixXf ki(3,3);

// SUSCRIBERS
ros::Subscriber image;
ros::Subscriber pose;

// PUBLISHERS
ros::Publisher ORBi;
ros::Publisher pose_estimation;
ros::Publisher path;
ros::Publisher odom;
ros::Publisher homop;

vector<Point2f> points1, points2; 
vector<uchar> status;

bool init = true;
nav_msgs::Path msg_path;
bool initz = false;
float z = 0;
float xa = 0, ya = 0, za = 0;
float min_high = 0;
int dif = 0;
float width = 0, height = 0;
float XY_homo = 0, XZ_homo = 0; 

Mat old_img, old_gray, frame_img, frame_gray;

vector<float> int_matrix(9);

void equalize_ycrcb(cv::Mat img)
{
    if(img.channels() >= 3)
    {
        cv::Mat ycrcb;

        cv::cvtColor(img,ycrcb,cv::COLOR_BGR2YCrCb);

        std::vector<cv::Mat> channels;
        cv::split(ycrcb,channels);

        cv::equalizeHist(channels[0], channels[0]);

        cv::Mat result;
        cv::merge(channels,ycrcb);

        cv::cvtColor(ycrcb,img,cv::COLOR_YCrCb2BGR);
    }
}

void poseCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    if(!takeoff)
    {
        geometry_msgs::PoseStamped m;

        z = msg->range; //msg->range

        m.pose.position.x = (XG + xa) / 2 * escala;
        m.pose.position.y = (YG + ya) / 2 * escala;
        m.pose.position.z = z;

        // Obtain a quaternion from the estimated yaw
        tf::Quaternion q_t = tf::createQuaternionFromRPY(0, 0, YAWG); //- XY_homo*3.1416/180
        geometry_msgs::Quaternion _q_pose;
        _q_pose.x = q_t.x();
        _q_pose.y = q_t.y();
        _q_pose.z = q_t.z();
        _q_pose.w = q_t.w();
        m.pose.orientation = _q_pose;

        if(msg->range == 0.0) //FALLO EN LA MEDIDA
        {
            cout << "ERROR EN LA LECTURA DE ALTÍMETRO" << endl;
            m.pose.position.x = xa;
            m.pose.position.y = ya;
            m.pose.position.z = za;
        }

        xa = m.pose.position.x;
        ya = m.pose.position.y;
        za = m.pose.position.z;

        float xrot = 0;
        float yrot = 0;
        float zrot = 0; 

        xrot = m.pose.position.x;
        yrot = m.pose.position.y;
        zrot = m.pose.position.z;

        // XY Rotation
        float xtrans = xrot;
        float ytrans = yrot;
        float theta = XY_homo * 3.1416/180; 
        xrot = xtrans*cos(theta) + ytrans*sin(theta);
        yrot = -xtrans*sin(theta) + ytrans*cos(theta); 

        // XZ Rotation
        xtrans = xrot;
        float ztrans = zrot;
        theta = XZ_homo * 3.1416/180;
        xrot = xtrans*cos(theta) + ztrans*sin(theta);
        zrot = -xtrans*sin(theta) + ztrans*cos(theta);

        m.pose.position.x = xrot;
        m.pose.position.y = yrot;
        m.pose.position.z = zrot;

        msg_path.poses.push_back(m);
        path.publish(msg_path);
        msg_path.header.seq++;

        nav_msgs::Odometry odom;
        odom.pose.pose = m.pose;
        odom.header.frame_id = odom_frame_id;
        homop.publish(odom);
    }
}

void imageCallback(const sensor_msgs::Image::ConstPtr& image)
{
    if(!takeoff)
    {
        ros::WallTime start_, end_;
        start_ = ros::WallTime::now();
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);  

        if(init)
        {
            old_img = cv_ptr->image;
            equalize_ycrcb(old_img);
            cvtColor(old_img, old_gray, COLOR_BGR2GRAY); 
            width = old_gray.cols;
            height = old_gray.rows;

            // int ncols = 2, nrows = 1;
            // boxFeature(old_gray, points1, ncols, nrows);
            goodFeaturesToTrack(old_gray, points1, init_points, edge_threshold, minDistance, Mat(), blockSize, false, responseK);

            drawFeatures(old_img, points1);

            init = false;
        }
        else
        {
            frame_img = cv_ptr->image;
            equalize_ycrcb(frame_img);
            cvtColor(frame_img, frame_gray, COLOR_BGR2GRAY);

            if (!old_gray.empty() && !frame_gray.empty()) 
            {
                featureTracking(old_gray, frame_gray, points1, points2, status); 
                Mat A(2*points1.size(), 4, CV_64FC1);
                Mat b(2*points1.size(), 1, CV_64FC1);
                for(int i = 0; i < points1.size(); i++)
                {
                    A.at<double>(i*2,0) = points1[i].x - width / 2; A.at<double>(i*2,1) = - (points1[i].y - height / 2); A.at<double>(i*2,2) = 1; A.at<double>(i*2,3) = 0; 
                    A.at<double>(i*2 + 1,0) = points1[i].y - height / 2; A.at<double>(i*2 + 1,1) = points1[i].x - width / 2; A.at<double>(i*2 + 1,2) = 0; A.at<double>(i*2 + 1,3) = 1; 
                    b.at<double>(i*2,0) = points2[i].x - width / 2;
                    b.at<double>(i*2 + 1,0) = points2[i].y - height / 2;
                }
                
                Eigen::MatrixXd Ae;
                Eigen::MatrixXd be;
                cv2eigen(A,Ae);
                cv2eigen(b,be);
                Eigen::VectorXd xopt = Ae.colPivHouseholderQr().solve(be);
                float u = xopt(2);
                float v = xopt(3);

                // cout << "theta: " << atan(xopt(1)/xopt(0))*180/CV_PI << endl;
                cout << "Px: " << u << endl;
                cout << "Py: " << v << endl;

                Eigen::VectorXd incI(3);
    
                incI << (ki(0,0)*u + ki(0,1)*v), //+ ki(0,2)
                        (ki(1,0)*u + ki(1,1)*v), //+ ki(1,2)
                        (ki(2,0)*u + ki(2,1)*v); //+ ki(2,2)

                float xi = incI[0];
                float yi = incI[1];
                // cout << "xi: " << xi << endl;
                // cout << "yi: " << yi << endl;            
                if(xi < 0.06 && yi < 0.06)
                {
                    YAWG += atan(xopt(1)/xopt(0));
                    XG = XG - (xi * cos(YAWG) + yi * sin(YAWG)) * z;
                    YG = YG + (-xi * sin(YAWG) + yi * cos(YAWG)) * z;
                }

                // cout << "points2:" << int(points2.size()) << " dif:" << dif << endl; // " n: " << n << 

                if (abs(int(points2.size()) - dif) > lost_points) // si la diferencia es menor a 
                {
                    // Puntos para el fragmento de la imagen
                    vector<Point2f> pointsf;
                    auto [ni, nj] = minFeatureBox(frame_gray, points2, ncols, nrows, advise_threshold);
                    cout << "nrow: " << ni << " ncol: " << nj << endl;
                    
                    if(ni != -1 && nj != -1) // Recalculo puntos si se han perdido
                    {
                        vector<Point2f> pointsf;
                        Mat box(height/nrows,width/ncols, CV_8UC1);

                        for(int i = 0; i < width/ncols; i++)
                        {    
                            for(int j = 0; j < height/nrows; j++)
                            {
                                box.at<uint8_t>(j, i) = frame_gray.at<uint8_t>(j+nj*height/nrows, i+ni*width/ncols); 
                            }
                        }

                        goodFeaturesToTrack(box, pointsf, new_points, edge_threshold, minDistance, Mat(), blockSize, false, responseK);

                        cout << pointsf.size() << endl;

                        for(int k = 0; k < pointsf.size(); k++)
                        {
                            pointsf[k].x = pointsf[k].x + ni*width/ncols;
                            pointsf[k].y = pointsf[k].y + nj*height/nrows;
                        }
                        points2.insert(points2.end(), pointsf.begin(), pointsf.end()); // Añado los nuevos puntos

                        dif = points2.size(); // Reiniciamos contador y puntos perdidos
                    }
                }

                end_ = ros::WallTime::now();
                double execution_time = (end_ - start_).toNSec() * 1e-6;
                // cout << execution_time << endl;    

                drawFeatures(cv_ptr->image, points2);
                ORBi.publish(cv_ptr->toImageMsg());

                old_gray = frame_gray.clone();
                old_img = frame_img.clone();

                if(points2.size() < min_points){
                    goodFeaturesToTrack(frame_gray, points1, init_points, edge_threshold, minDistance, Mat(), blockSize, false, responseK);
                } 
                else{
                    points1 = points2;
                }

                points2.clear();       
            }
        }
    }
}

bool node_init(mission_manager::mission_manager::Request  &req, mission_manager::mission_manager::Response &res)
{
    cout << "[CAMERA]: service received" << endl;
    XG = req.x_pos;
    YG = req.y_pos;
    //YAWG = 0.0;
    YAWG = req.yaw_rot;
    //YAWG = -XY_homo*3.1416/180;
    xa = XG;
    ya = YG;

    takeoff = req.takeoff;
    return true;
}

int main(int argc, char **argv)
{
    ROS_INFO("Starting Homography node with camera info...");
    string _node_name = "visual_odometry" ; //homography
    ros::init(argc, argv, _node_name);
    ros::NodeHandle n;
    msg_path.header.frame_id = "/base_link_init";

    //Filter parameters
    n.getParam(_node_name + "/MIN_POINTS", min_points);  //
    n.getParam(_node_name + "/NEW_POINTS", new_points);	 // 
    n.getParam(_node_name + "/INITIAL_POINTS", init_points);  //  
    n.getParam(_node_name + "/LOST_POINTS", lost_points);  //
    n.getParam(_node_name + "/EDGE_THRESHOLD", edge_threshold);  //
    n.getParam(_node_name + "/NROWS", nrows);  //
    n.getParam(_node_name + "/NCOLS", ncols);  // 
    n.getParam(_node_name + "/ADVISE_THRESHOLD", advise_threshold);  //
    n.getParam(_node_name + "/MININUM_DISTANCE", minDistance);  //
    n.getParam(_node_name + "/BLOCK_SIZE", blockSize);
    n.getParam(_node_name + "/RESPONSE_K", responseK);
    n.getParam(_node_name + "/SCALE",escala);  // 
    n.getParam(_node_name + "/HIGH",min_high);  // 
    n.getParam(_node_name + "/ODOM_FRAME_ID",odom_frame_id);  // 
    n.getParam(_node_name + "/ROT_XY",XY_homo); // 
    n.getParam(_node_name + "/ROT_XZ",XZ_homo); // 
    n.getParam(_node_name + "/INTRINSIC_PARAMETERS",int_matrix); // 
    n.getParam(_node_name + "/DEBUG",debug); // Debug mode
    n.getParam(_node_name + "/IMAGE_TOPIC",image_topic); 
    n.getParam(_node_name + "/ALTIMETER_TOPIC",altimeter_topic); 
    n.getParam(_node_name + "/PATCH_TOPIC",path_topic); 
    n.getParam(_node_name + "/POSE_TOPIC",pose_topic); 
    n.getParam(_node_name + "/ODOM_TOPIC",odom_topic); 
    n.getParam(_node_name + "/ORB_IMAGE_TOPIC",imageORB_topic); 
    n.getParam(_node_name + "/MISSION_MANAGER_SERVICE",mission_manager_service); 
    
    // Matriz de parametros intrínsecos invertida
    ki << int_matrix[0], int_matrix[1], int_matrix[2],
          int_matrix[3], int_matrix[4], int_matrix[5],
          int_matrix[6], int_matrix[7], int_matrix[8];

    image = n.subscribe(image_topic, 100, imageCallback);
    pose = n.subscribe(altimeter_topic, 100, poseCallback); 

    ORBi = n.advertise<sensor_msgs::Image>(imageORB_topic, 100);  
    path = n.advertise<nav_msgs::Path>(path_topic, 100); 
    odom = n.advertise<geometry_msgs::PoseStamped>(odom_topic, 100); 
    homop = n.advertise<nav_msgs::Odometry>(pose_topic, 100); 

    ros::ServiceServer service = n.advertiseService(mission_manager_service, node_init);

    ROS_INFO("Waiting images...");
    ros::spin();
}
