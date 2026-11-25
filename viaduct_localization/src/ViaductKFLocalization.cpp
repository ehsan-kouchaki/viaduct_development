/**
*   @file       ViaductKFLocalization.cpp
*
*   @author     Maria Polvillo Diaz <mapodi.94@gmail.com>
*   @memberof   GRVC
*
*   @date       13/06/2022
*   @brief      Definition of KFLocalization class
*/

#include "ViaductKFLocalization.h"

/**
      * @brief Constructor of the KFLocalization class
      * @param _node ros node handler      
*/
KFLocalization::KFLocalization(ros::NodeHandle &_node, tf::TransformBroadcaster *tfbr_)
{
    this->node_ = _node; // ros node handle
    first_time_ = true;  // until first_time flag is false, the filter is not initialized
    //tag_found_ = false;       // initially no tag sensor has been found
    first_meas_ = true;        // until the first_meas is true, the local position is not published
    x_0_ = y_0_ = z_0_ = 0.0f; // initial estimated position of the drone
    takeoff_stage_ = true;
    br_ = tfbr_;
    lidar_received_ = false; // if first lidar measurement has been received

    // Get parameters
    getParameters();

    // Definition of ROS Subscribers
    /* dist_sub_ = node_.subscribe(beacon_topic_, 1,
                                &KFLocalization::ekfCB, this); */
    gpspos_sub_ = node_.subscribe<nav_msgs::Odometry>(gpspos_topic_, 1,
                                                      [this](const nav_msgs::Odometry::ConstPtr &_msg) {
                                                          gps_pos_ = *_msg;
                                                      });
    ceildis_sub_ = node_.subscribe<std_msgs::Float32>("/ceil_distance", 1,
                                                      [this](const std_msgs::Float32::ConstPtr &_msg) {
                                                          ceil_distance_ = _msg->data;
                                                      });
    lidarpos_sub_ = node_.subscribe<nav_msgs::Odometry>(lidarpos_topic_, 1,
                                                        [this](const nav_msgs::Odometry::ConstPtr &_msg) {
                                                            lidar_pos_ = *_msg;

                                                            // Get yaw angle from localization data
                                                            geometry_msgs::Quaternion _q = _msg->pose.pose.orientation;
                                                            tf::Quaternion _q_lidar(_q.x, _q.y, _q.z, _q.w);
                                                            double _roll_lidar, _pitch_lidar;
                                                            tf::Matrix3x3(_q_lidar).getRPY(_roll_lidar, _pitch_lidar, yaw_lidar_);
                                                            lidar_received_ = true;
                                                        });
    homogpos_sub_ = node_.subscribe<nav_msgs::Odometry>(homogpos_topic_, 1,
                                                        [this](const nav_msgs::Odometry::ConstPtr &_msg) {
                                                            homog_pos_ = *_msg;

                                                            // Get yaw angle from localization data
                                                            geometry_msgs::Quaternion _q = _msg->pose.pose.orientation;
                                                            tf::Quaternion _q_homog(_q.x, _q.y, _q.z, _q.w);
                                                            double _roll_homog, _pitch_homog;
                                                            tf::Matrix3x3(_q_homog).getRPY(_roll_homog, _pitch_homog, yaw_homog_);
                                                        });
    homogdelta_sub_ = node_.subscribe<geometry_msgs::PoseStamped>(homogdelta_topic_, 1,
                                                        [this](const geometry_msgs::PoseStamped::ConstPtr &_msg) {
                                                            deltapos_homog_ = _msg->pose.position;
                                                            // ROS_INFO("[DEBUG]: dpos_homog = (%f, %f, %f)", deltapos_homog_.x, deltapos_homog_.y, deltapos_homog_.z);

                                                            // Get increment in yaw angle from homography method information
                                                            geometry_msgs::Quaternion _q = _msg->pose.orientation;
                                                            tf::Quaternion _deltaq_homog(_q.x, _q.y, _q.z, _q.w);
                                                            double _deltaroll_homog, _deltapitch_homog;
                                                            tf::Matrix3x3(_deltaq_homog).getRPY(_deltaroll_homog, _deltapitch_homog, deltayaw_homog_); // TODO: check if translating deltaq is correct
                                                            // ROS_INFO("[DEBUG]: deltayaw_homog_ = %f", deltayaw_homog_);
                                                        });                                                        
    imu_sub_ = node_.subscribe<sensor_msgs::Imu>(imu_topic_, 1,
                                                 [this](const sensor_msgs::Imu::ConstPtr &_msg) {
                                                     // Get yaw angle from IMU data
                                                     geometry_msgs::Quaternion _q = _msg->orientation;
                                                     tf::Quaternion _q_imu(_q.x, _q.y, _q.z, _q.w);
                                                     tf::Matrix3x3(_q_imu).getRPY(roll_imu_, pitch_imu_, yaw_imu_);
                                                     linear_acc_ = _msg->linear_acceleration;
                                                 });

    altim_sub_ = node_.subscribe(altimeter_topic_, 1, &KFLocalization::kfCB, this);

    iniyaw_sub_ = node_.subscribe<geometry_msgs::QuaternionStamped>(iniyaw_topic_, 1,
                                                                    [this](const geometry_msgs::QuaternionStamped::ConstPtr &_msg) {
                                                                        // Get yaw angle from measurements
                                                                        geometry_msgs::Quaternion _q_msg = _msg->quaternion;
                                                                        tf::Quaternion _q_ini(_q_msg.x, _q_msg.y, _q_msg.z, _q_msg.w);
                                                                        double _roll_ini, _pitch_ini;
                                                                        tf::Matrix3x3(_q_ini).getRPY(_roll_ini, _pitch_ini, yaw_ini_);
                                                                    });

    ceilingpos_sub_ = node_.subscribe<nav_msgs::Odometry>(ceiling_pose_topic_, 1,
                                                          [this](const nav_msgs::Odometry::ConstPtr &_msg) {
                                                              ceil_pos_ = *_msg;
                                                          });

    // TODO: hacer que el filtro se ejecute a una cierta tasa especificada

    // Definition of ROS Publishers
    state_pub_ = node_.advertise<nav_msgs::Odometry>(est_pose_topic_, 1);

    // Definition of service server
    service_ = node_.advertiseService("/mission_manager", &KFLocalization::node_init, this);

    // Initalization of variables of the filter
    B_ = Eigen::VectorXd::Zero(4);
    A_ = Eigen::MatrixXd::Identity(4, 4);
    I_ = Eigen::MatrixXd::Identity(4, 4);
    R_ = Eigen::MatrixXd::Zero(4, 4);
    Qtakeoff_ = Eigen::MatrixXd::Zero(4, 4);
    Qnav_ = Eigen::MatrixXd::Zero(4, 4);
    mut_ = Eigen::VectorXd::Zero(4);
    mut1_ = Eigen::VectorXd::Zero(4);
    pmut_ = Eigen::VectorXd::Zero(4);
    sigmat_ = Eigen::MatrixXd::Zero(4, 4);
    sigmat1_ = Eigen::MatrixXd::Zero(4, 4);
    psigmat_ = Eigen::MatrixXd::Zero(4, 4);
    Ct_ = Eigen::MatrixXd::Identity(4, 4);
    kt_ = Eigen::MatrixXd::Zero(4, 4);
    zt_ = Eigen::VectorXd::Zero(4);

    // Initializatio of the variance matrix of the prediction stage
    //R_ = pow(sigma_model_, 2) * I_;
    R_(0, 0) = pow(sigma_homog_[0], 2);
    R_(1, 1) = pow(sigma_homog_[1], 2);
    R_(2, 2) = pow(sigma_homog_[2], 2);
    R_(3, 3) = pow(sigma_homog_[3], 2);

    // Initialization of the variance matrix of the observation model
    // It is supposed that in the initial stage, the UAV havent taken off yet

    //Qtakeoff_(0, 0) = pow(sigma_gps_[0], 2);
    // Qtakeoff_(0, 0) = pow(sigma_ceilpos_, 2);
    // Qtakeoff_(1, 1) = pow(sigma_gps_[1], 2);
    // Qtakeoff_(2, 2) = pow(sigma_altim_, 2);
    // Qtakeoff_(3, 3) = pow(sigma_homog_[1], 2);

    // Qnav_(0, 0) = pow(sigma_lidar_[0], 2);
    // Qnav_(1, 1) = pow(sigma_homog_[0], 2);
    // Qnav_(2, 2) = pow(sigma_lidar_[1], 2);
    // Qnav_(3, 3) = pow(sigma_homog_[1], 2);

    // La fase de actualizacion depende unicamente del LiDAR
    Qtakeoff_(0, 0) = pow(sigma_lidar_[0], 2);
    Qtakeoff_(1, 1) = pow(sigma_lidar_[1], 2);
    Qtakeoff_(2, 2) = pow(sigma_lidar_[2], 2);
    Qtakeoff_(3, 3) = pow(sigma_lidar_[3], 2);

    Qnav_(0, 0) = pow(sigma_lidar_[0], 2);
    Qnav_(1, 1) = pow(sigma_lidar_[1], 2);
    Qnav_(2, 2) = pow(sigma_lidar_[2], 2);
    Qnav_(3, 3) = pow(sigma_lidar_[3], 2);
}

/**
      * @brief Destructor of the KFLocalization class
*/
KFLocalization::~KFLocalization(){
    ROS_INFO("[viaduct_kf_localization]: Node has been killed");
}

/**
      * @brief Retrieve parameters to configure the EKF filter
*/
void KFLocalization::getParameters(){
    // Read parameters. Fully documented in yaml configuration file
    iniposition_[3];
    if(!node_.getParam("iniposition", iniposition_)) iniposition_.assign(3, 0.0);
    sigma_gps_[2];
    if(!node_.getParam("sigma_gps", sigma_gps_)) sigma_gps_.assign(2, 0.3);
    sigma_lidar_[4];
    if(!node_.getParam("sigma_lidar", sigma_lidar_)) sigma_lidar_.assign(4, 0.015);
    sigma_homog_[4];
    if(!node_.getParam("sigma_homog", sigma_homog_)) sigma_homog_.assign(4, 0.3);
    if(!node_.getParam("gpspos_topic", gpspos_topic_)){
        gpspos_topic_ = GPS_POS_TOPIC_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param gpspos_topic set to DEAFULT");
    }
    if(!node_.getParam("lidarpos_topic", lidarpos_topic_)){
        lidarpos_topic_ = LIDAR_POS_TOPIC_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param lidarpos_topic set to DEAFULT");
    }
    if(!node_.getParam("homogpos_topic", homogpos_topic_)){
        homogpos_topic_ = HOMOG_POS_TOPIC_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param homogpos_topic set to DEAFULT");
    }
    if(!node_.getParam("homogdelta_topic", homogdelta_topic_)){
        homogdelta_topic_ = HOMOG_DELTA_TOPIC_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param homogdelta_topic set to DEAFULT");
    }
    if(!node_.getParam("imu_topic", imu_topic_)){
        imu_topic_ = IMU_TOPIC_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param imu_topic set to DEAFULT");
    }
    if(!node_.getParam("altimeter_topic", altimeter_topic_)){
        altimeter_topic_ = ALTIMETER_TOPIC_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param altimeter_topic set to DEAFULT");
    }
    if(!node_.getParam("iniyaw_topic", iniyaw_topic_)){
        iniyaw_topic_ = INIYAW_TOPIC_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param iniyaw_topic set to DEAFULT");
    }
    if(!node_.getParam("ceiling_pose_topic", ceiling_pose_topic_)){
        ceiling_pose_topic_ = CEILINGPOSE_TOPIC_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param ceiling_pose_topic set to DEAFULT");
    }
    if(!node_.getParam("est_pose_topic", est_pose_topic_)){
        est_pose_topic_ = EST_POSE_TOPIC_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param est_pose_topic set to DEAFULT");
    }
    if(!node_.getParam("frame_id", frame_id_)){
        frame_id_ = FRAME_ID_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param frame_id set to DEAFULT");
    }
    if(!node_.getParam("child_frame_id", child_frame_id_)){
        child_frame_id_ = CHILD_FRAME_ID_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param child_frame_id set to DEAFULT");
    }
    if(!node_.getParam("sigma_model", sigma_model_)){
        sigma_model_ = SIGMAMODEL_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param sigma_model set to DEAFULT");
    }
    if(!node_.getParam("sigma_altim", sigma_altim_)){
        sigma_altim_ = SIGMAALTIMETER_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param sigma_altim set to DEAFULT");
    }
    if(!node_.getParam("sigma_ceilpos", sigma_ceilpos_)){
        sigma_ceilpos_ = SIGMACEILPOS_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param sigma_ceilpos set to DEAFULT");
    }
    if(!node_.getParam("contrib_module", contrib_module_)){
        contrib_module_ = CONTRIBMODULE_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param contrib_module set to DEAFULT");
    }
    if(!node_.getParam("publish_tf", publish_tf_)){
        publish_tf_ = PUBLISHTF_DEFAULT;
        ROS_INFO("[viaduct_kf_localization]: Param publish_tf set to DEAFULT");
    }
    if(!node_.getParam("time_threshold", time_threshold_)){
        time_threshold_ = TIMETHRESHOLD_DEFAULT;
        ROS_INFO("[beacon_ekf_localization]: Param time_threshold set to DEAFULT");
    }
    if(!node_.getParam("filter_outliers", filter_outliers_)){
        filter_outliers_ = FILTEROUTLIERS_DEFAULT;
        ROS_INFO("[beacon_ekf_localization]: Param filter_outliers_ set to DEAFULT");
    }
    if(!node_.getParam("deltam_outlier", deltam_outlier_)){
        deltam_outlier_ = DELTAOUTLIER_DEFAULT;
        ROS_INFO("[beacon_ekf_localization]: Param deltam_outlier_ set to DEAFULT");
    }

    // TODO: modify depending on the convergence of GPS,
    // or shadows or the amount of points the LIDAR detects ?

    // Print parameters
    ROS_INFO("\n Parameters: \n"
             "      iniposition_ = [%f, %f, %f] \n"
             "      gpspos_topic = %s \n"
             "      lidarpos_topic = %s \n"
             "      homogpos_topic = %s \n"
             "      homogdelta_topic = %s \n"
             "      imu_topic = %s \n"
             "      altimeter_topic = %s \n"
             "      iniyaw_topic = %s \n"
             "      ceiling_pose_topic = %s \n"
             "      est_pose_topic = %s \n"
             "      frame_id = %s \n"
             "      child_frame_id = %s \n"
             "      sigma_model = %f \n"
             "      sigma_gps = [%f, %f] \n"
             "      sigma_lidar = [%f, %f] \n"
             "      sigma_homog = [%f, %f] \n"
             "      sigma_altim = %f \n"
             "      sigma_ceilpos = %f \n"             
             "      contrib_module = %f \n"
             "      publish_tf = %i \n"
             "      time_threshold = %f \n"
             "      filter_outliers = %i \n"
             "      deltam_outlier = %f \n",
             iniposition_[0], iniposition_[1], iniposition_[2],
             gpspos_topic_.c_str(), lidarpos_topic_.c_str(), homogpos_topic_.c_str(), 
             homogdelta_topic_.c_str(), imu_topic_.c_str(), altimeter_topic_.c_str(),
             iniyaw_topic_.c_str(), ceiling_pose_topic_.c_str(), est_pose_topic_.c_str(),
             frame_id_.c_str(), child_frame_id_.c_str(), sigma_model_, sigma_gps_[0],
             sigma_gps_[1], sigma_lidar_[0], sigma_lidar_[1], sigma_homog_[0], 
             sigma_homog_[1], sigma_altim_, sigma_ceilpos_, contrib_module_, 
             publish_tf_, time_threshold_, filter_outliers_, deltam_outlier_);

    ROS_INFO("\n Parameters and beacon map received \n");
}

bool KFLocalization::node_init(mission_manager::mission_manager::Request &req,
                               mission_manager::mission_manager::Response &res)
{
    takeoff_stage_ = false;
    std::cerr << "[DEBUG]: ---------- SRV TRIGGER ---------- " << std::endl;
    // std::cerr << "[DEBUG]: x = " << req.x_pos << std::endl;

    return true;
}

/**
      * @brief EKF filter
      * @param _node_info 
*/
void KFLocalization::kfCB(const sensor_msgs::Range::ConstPtr &_msg) //SI NO FUNCIONA ALTIMETER?
{
    altim_height_ = _msg->range;

    if (first_time_) // filter initialization
    {
        // First position is settted by parameters
        mut1_(0) = iniposition_[0];
        mut1_(1) = iniposition_[1];
        mut1_(2) = iniposition_[2];
        mut1_(3) = yaw_ini_; //0.0; //1.571;

        sigmat1_ = R_; // TODO: probar con inicializar con zeros

        curr_t_.stamp = ros::Time::now();
        t1_ = curr_t_.stamp.sec + curr_t_.stamp.nsec * pow(10, -9);

        first_time_ = false;
    }
    else
    {
        // *** Prediction Stage ***
        curr_t_.stamp = ros::Time::now();
        t_ = curr_t_.stamp.sec + curr_t_.stamp.nsec * pow(10, -9);

        // If the difference between timestamps of two consecutive measurements is
        // greater than a threshold, then the control matrix is zero
        double _diff_t = t_ - t1_;
        if (_diff_t > time_threshold_)
            B_ = Eigen::VectorXd::Zero(3);
        else
        {
            double t_square = pow(_diff_t, 2);
            B_(0) = t_square * linear_acc_.x;
            B_(1) = t_square * linear_acc_.y;
            B_(2) = t_square * linear_acc_.z; // deltayaw_homog_, deltapos_homog_
        }

        if (takeoff_stage_)
            pmut_ = A_ * mut1_ + B_;          // No reliable data from homography method
        else
        {
            // AÑADIR A deltapos_homog_ EL GIRO CONVENIENTE USANDO  mut_(3)

            pmut_(0) = mut1_(0) + deltapos_homog_.y;   // homog_pos_.pose.pose.position.x
            pmut_(1) = mut1_(1) + deltapos_homog_.x;   // homog_pos_.pose.pose.position.y
            pmut_(2) = mut1_(2) + deltapos_homog_.z;   // homog_pos_.pose.pose.position.z
            ROS_INFO("[DEBUG]: delta_pos_hog = %f, %f, %f,", deltapos_homog_.x, deltapos_homog_.y, deltapos_homog_.z);
            pmut_(3) = mut1_(3) + deltayaw_homog_;     // yaw_homog_
        }

        if(ceil_distance_ < 0.5){
            R_(0, 0) = pow(0.1*sigma_homog_[0], 2);
            R_(1, 1) = pow(0.1*sigma_homog_[1], 2);
            R_(2, 2) = pow(0.1*sigma_homog_[2], 2);
            R_(3, 3) = pow(0.1*sigma_homog_[3], 2);

            Qnav_(0, 0) = pow(10*sigma_lidar_[0], 2);
            Qnav_(1, 1) = pow(10*sigma_lidar_[1], 2);
            Qnav_(2, 2) = pow(10*sigma_lidar_[2], 2);
            Qnav_(3, 3) = pow(10*sigma_lidar_[3], 2);
        }else{
            R_(0, 0) = pow(sigma_homog_[0], 2);
            R_(1, 1) = pow(sigma_homog_[1], 2);
            R_(2, 2) = pow(sigma_homog_[2], 2);
            R_(3, 3) = pow(sigma_homog_[3], 2);

            Qnav_(0, 0) = pow(sigma_lidar_[0], 2);
            Qnav_(1, 1) = pow(sigma_lidar_[1], 2);
            Qnav_(2, 2) = pow(sigma_lidar_[2], 2);
            Qnav_(3, 3) = pow(sigma_lidar_[3], 2);
        }

            // Imprimir los valores de ceil_distance_, Q_nav_ y R_
        std::cout << "ceil_distance_: " << ceil_distance_ << std::endl;
        // std::cout << "Q_nav_: " << Qnav_(0, 0) << std::endl; // Asegúrate de tener definida la variable Q_nav_
        // std::cout << "R_: " << R_(0, 0) << std::endl; // Asegúrate de tener definida la variable R_

        //pmut_ = A_ * mut1_ + B_;
        psigmat_ = A_ * sigmat1_ * A_.transpose() + R_; // TODO: R no se actualiza ??

        // *** Update Stage ***
        Eigen::MatrixXd _mat_aux2;
        Eigen::MatrixXd _mat_inv;

        if (takeoff_stage_)
            _mat_aux2 = Ct_ * psigmat_ * Ct_.transpose() + Qtakeoff_;
        else
            _mat_aux2 = Ct_ * psigmat_ * Ct_.transpose() + Qnav_;

        if (_mat_aux2.determinant() == 0)
        {
            ROS_ERROR("The matrix is not invertible \n");
            return;
        }
        else
            _mat_inv = _mat_aux2.inverse();

        kt_ = psigmat_ * Ct_.transpose() * _mat_inv;

        // Reading measurement
        if (takeoff_stage_)
        {
            /*zt_(0) = -ceil_pos_.pose.pose.position.y; //gps_pos_.pose.pose.position.x;
            zt_(1) = gps_pos_.pose.pose.position.y;
            zt_(2) = altim_height_; //lidar_pos_.pose.pose.position.z;
            zt_(3) = yaw_ini_;      //0.0           //-1.571;           //yaw_imu_;*/
            std::cerr << "[DEBUG]: Takeoff stage " << std::endl;
	        zt_(0) = lidar_pos_.pose.pose.position.x;
            zt_(1) = lidar_pos_.pose.pose.position.y;
            zt_(2) = lidar_pos_.pose.pose.position.z;
            //zt_(2) = altim_height_;
            zt_(3) = yaw_lidar_; //yaw_homog_
        }
        else
        {
            if (lidar_received_)
            {
                zt_(0) = lidar_pos_.pose.pose.position.x;
                zt_(1) = lidar_pos_.pose.pose.position.y;
                zt_(2) = lidar_pos_.pose.pose.position.z;
                //zt_(2) = altim_height_;
                zt_(3) = yaw_lidar_; //yaw_homog_
            }
            else    // TODO: check this!! ESTA MAL
            {
                zt_(0) = -ceil_pos_.pose.pose.position.y; //gps_pos_.pose.pose.position.x;
                zt_(1) = gps_pos_.pose.pose.position.y;
                zt_(2) = altim_height_; //lidar_pos_.pose.pose.position.z;
                zt_(3) = yaw_ini_;      // yaw_homog; //0.0
            }

            std::cerr << "[DEBUG]: Navigation stage " << std::endl;
            //std::cerr << "[DEBUG]: zt_ = " << zt_ << std::endl;
        }

        Eigen::VectorXd _contrib;
        _contrib = (zt_ - Ct_ * pmut_); // TODO: check this!!
        std::cerr << "[DEBUG]: _contrib = " << _contrib << std::endl;
        double _contrib_module = sqrt(pow(_contrib(0), 2) + pow(_contrib[1], 2) + pow(_contrib[2], 2));
        std::cerr << "[DEBUG]: _contrib_module = " << _contrib_module << std::endl;

        mut_ = pmut_ + kt_ * (zt_ - Ct_ * pmut_); // TODO: esto es contrib
        sigmat_ = (I_ - kt_ * Ct_) * psigmat_;

        /* if (_contrib_module > contrib_module_)
        {            
            mut_ = pmut_;
            sigmat_ = psigmat_;
        } */

        // Obtain a quaternion from the estimated yaw
        tf::Quaternion q_t = tf::createQuaternionFromRPY(roll_imu_, pitch_imu_, mut_(3));
        geometry_msgs::Quaternion _q_pose;
        _q_pose.x = q_t.x();
        _q_pose.y = q_t.y();
        _q_pose.z = q_t.z();
        _q_pose.w = q_t.w();

        // Publish the state vector
        nav_msgs::Odometry _state;
        _state.header.frame_id = frame_id_;
        _state.child_frame_id = child_frame_id_;
        _state.pose.pose.orientation = _q_pose;

        // Filter of outliers
        if (filter_outliers_)
        {
            double _dist_m = sqrt(pow(mut_(0) - mut1_(0), 2) + pow(mut_(1) - mut1_(1), 2) + pow(mut_(2) - mut1_(2), 2));
            //std::cerr << "[DEBUG]: _dist_m = " << _dist_m << std::endl;
            if (_dist_m > deltam_outlier_)
            {
                _state.pose.pose.position.x = mut1_(0);
                _state.pose.pose.position.y = mut1_(1);
                _state.pose.pose.position.z = mut1_(2);

                /* std::cerr << "[DEBUG]: ----- Entro en IF ------ " << std::endl;
                std::cout << "mut_" << mut_ << std::endl;
                std::cout << "mut1_" << mut1_ << std::endl; */
            }
            else
            {
                _state.pose.pose.position.x = mut_(0);
                _state.pose.pose.position.y = mut_(1);
                _state.pose.pose.position.z = mut_(2);
            }
        } //TODO: comprobar si esto vale con el KF

        _state.header.stamp = ros::Time::now();
        state_pub_.publish(_state);

        if (publish_tf_)
        {
            tf::StampedTransform _tf_loc;
            _tf_loc.setOrigin(tf::Vector3(mut_(0), mut_(1), mut_(2)));
            _tf_loc.setRotation(q_t); //tf::Quaternion(fYaw, fPitch, fRoll)
            br_->sendTransform(tf::StampedTransform(_tf_loc, ros::Time::now(), frame_id_, child_frame_id_));
        }

        // TODO: rellenar el roll y pitch con la IMU
        // TODO: comprobar con qué se realimenta el yaw, si con IMU o sólo con homog

        /* if (first_meas_)
        {
            x_0_ = mut_(0);
            y_0_ = mut_(1);
            z_0_ = mut_(2);
            first_meas_ = false;
        }
        else
        {
            _state.pose.pose.position.x = mut_(0) - x_0_;
            _state.pose.pose.position.y = mut_(1) - y_0_;
            _state.pose.pose.position.z = mut_(2) - z_0_;
            _state.header.stamp = ros::Time::now();
            local_state_pub_.publish(_state);
        } */

        // Update variables for the next iteration
        mut1_ = mut_;
        sigmat1_ = sigmat_;
        t1_ = t_;
    }
}

/**
      * @brief ROS spin loop
*/
void KFLocalization::spin()
{
    ros::spin();
}
