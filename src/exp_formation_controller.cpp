#include "exp_formation_controller.h"

ExpFormationController::ExpFormationController(
    const ros::NodeHandle & nh, const ros::NodeHandle & pnh)
    :nh_(nh),
    pnh_(pnh),
    exp_uav2_(ros::NodeHandle("uav2"), pnh_),
    exp_uav3_(ros::NodeHandle("uav3"), pnh_),
    exp_uav4_(ros::NodeHandle("uav4"), pnh_),
    ready_to_init_position_(true),
    ready_to_formation_(false),
    uav4_ready_to_fly_(false),
    ready_to_yaw_(false){
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        
        initializeState();
        initCenterOfCoordinate();
        loadParameter();
        setMode();

        ready_to_formation_sub_ = nh_.subscribe("/ready_to_formation", 1,
            &ExpFormationController::readyToFormationCallback, this);

        uav4_ready_to_fly_sub_ = nh_.subscribe("/uav4_ready_to_fly", 1,
            &ExpFormationController::uav4ReadyToFlyCallback, this);

        ready_to_yaw_sub_ = nh_.subscribe("/ready_to_yaw", 1, 
            &ExpFormationController::readyToYawCallback, this);

        inter_distance_error_pub_ = nh_.advertise<nav_msgs::Odometry>(
            "/inter_distance", 1);

        main_loop_timer_ = nh_.createTimer(ros::Duration(0.05), 
        &ExpFormationController::mainloop, this);
}
ExpFormationController::~ExpFormationController(){}

void ExpFormationController::mainloop(const ros::TimerEvent & time){
    pos_uav2_ = exp_uav2_.pos_;
    pos_uav3_ = exp_uav3_.pos_;
    pos_uav4_ = exp_uav4_.pos_;

    if(ready_to_init_position_){
        ROS_INFO_ONCE("Enter the process of sending the init position.");
        ROS_DEBUG("uav2_init_pos_/x: %f, y: %f, z: %f", uav2_init_pos_.x(), uav2_init_pos_.y(), uav2_init_pos_.z());
        ROS_DEBUG("uav3_init_pos_/x: %f, y: %f, z: %f", uav3_init_pos_.x(), uav3_init_pos_.y(), uav3_init_pos_.z());
        ROS_DEBUG("uav4_init_pos_/x: %f, y: %f, z: %f", uav4_init_pos_.x(), uav4_init_pos_.y(), uav4_init_pos_.z());

        exp_uav2_.pubLocalPosition(uav2_init_pos_);
        exp_uav3_.pubLocalPosition(uav3_init_pos_);
        exp_uav4_.pubLocalPosition(uav4_init_pos_);
    }

    if(ready_to_formation_){
        ROS_INFO_ONCE("Enter the formation");
        computeInterDistance();
        Eigen::Vector2d u2 = computeUav2VelCmd();
        Eigen::Vector2d u3 = computeUav3VelCmd();
        exp_uav2_.pubLocalCmdVel(u2, uav2_init_pos_[2]);
        exp_uav3_.pubLocalCmdVel(u3, uav3_init_pos_[2]);

        ROS_DEBUG("u2/x: %f, y: %f", u2.x(), u2.y());
        ROS_DEBUG("u3/x: %f, y: %f", u3.x(), u3.y());
        ROS_DEBUG("uav4_init_pos_/x: %f, y: %f, z: %f", uav4_init_pos_.x(), uav4_init_pos_.y(), uav4_init_pos_.z());
        //hold the uav4 hovering
        exp_uav4_.pubLocalPosition(uav4_init_pos_);
    }

    if(uav4_ready_to_fly_){
        ROS_INFO_ONCE("Enter the uav4");
        Eigen::Vector2d u4;
        u4[0] = 0.04 * (uav4_des_pos_.x() - pos_uav4_.x());
        u4[1] = 0.04 * (uav4_des_pos_.y() - pos_uav4_.y());
        ROS_DEBUG("uav4_des_pos_/x: %f, y: %f", uav4_des_pos_.x(), uav4_des_pos_.y());
        exp_uav4_.pubLocalCmdVel(u4, uav4_init_pos_[2]);
        //hold the uav2/uav3 hovering
        ROS_DEBUG("u4/x: %f, y: %f", u4.x(), u4.y());
        ROS_DEBUG("uav2_des_pos_/x: %f, y: %f, z: %f", uav2_des_pos_.x(), uav2_des_pos_.y(), uav2_des_pos_.z());
        ROS_DEBUG("relative uav2_des_pos_/x: %f, y: %f, z: %f", uav2_des_pos_.x() - pos_uav1_.x(), uav2_des_pos_.y() - pos_uav1_.y(), uav2_des_pos_.z());
        ROS_DEBUG("uav3_des_pos_/x: %f, y: %f, z: %f", uav3_des_pos_.x(), uav3_des_pos_.y(), uav3_des_pos_.z());
        ROS_DEBUG("relative uav3_des_pos_/x: %f, y: %f, z: %f", uav3_des_pos_.x() - pos_uav1_.x(), uav3_des_pos_.y() - pos_uav1_.y(), uav3_des_pos_.z());
        exp_uav2_.pubLocalPosition(uav2_des_pos_);
        exp_uav3_.pubLocalPosition(uav3_des_pos_);
    }

    if(ready_to_yaw_){
        ROS_INFO_ONCE("Enter the yaw");
        //需要测试此时是否需要发送位置指令进而保证无人机的悬停
        ROS_DEBUG("uav2_des_yaw:%f,uav3_des_yaw:%f,uav4_des_yaw:%f", des_yaw_.x(), des_yaw_.y(), des_yaw_.z());
        double use_cmd = true;
        Eigen::Vector3d uav4_des_pos;
        uav4_des_pos.x() = uav4_des_pos_.x();
        uav4_des_pos.y() = uav4_des_pos_.y();
        uav4_des_pos.z() = uav4_init_pos_.z();
        if(use_cmd){
            exp_uav2_.pubYawCmdVel(des_yaw_.x());
            exp_uav3_.pubYawCmdVel(des_yaw_.y());
            exp_uav4_.pubYawCmdVel(des_yaw_.z());
        }
        else{
            exp_uav2_.pubYawCmdPos(uav2_des_pos_, des_yaw_.x());
            exp_uav3_.pubYawCmdPos(uav3_des_pos_, des_yaw_.y());
            exp_uav4_.pubYawCmdPos(uav4_des_pos, des_yaw_.z());
        }
        
        
    }
}

void ExpFormationController::computeInterDistance(){
    double inter_dis_12 = (pos_uav1_ - pos_uav2_).norm();
    double inter_dis_13 = (pos_uav1_ - pos_uav3_).norm();
    double inter_dis_23 = (pos_uav2_ - pos_uav3_).norm();

    Eigen::Vector3d inter_distance_ = Eigen::Vector3d(inter_dis_12,
                                                                           inter_dis_13,
                                                                           inter_dis_23);

    inter_distance_error_ = inter_distance_.cwiseProduct(inter_distance_)
                                                 - des_inter_distance_.cwiseProduct(des_inter_distance_);

    if(inter_distance_error_.cwiseAbs().minCoeff() < threshold_){
        ROS_INFO_ONCE("The coefficient is %f", gain_m_);
        coefficient_ = gain_m_;
    }else{
        ROS_INFO_ONCE("The coefficient is %f", gain_m_);
        coefficient_ = gain_n_;
    }

    Eigen::Vector3d inter_distance_error_sqrt = inter_distance_ - des_inter_distance_;
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = inter_distance_error_sqrt.x();
    msg.pose.pose.position.y = inter_distance_error_sqrt.y();
    msg.pose.pose.position.z = inter_distance_error_sqrt.z();

    msg.twist.twist.linear = quadrotor_common::eigenToGeometry(inter_distance_);
    Eigen::Vector3d des_yaw_msg;
    des_yaw_msg = des_yaw_ / M_PI * 180.0;
    msg.twist.twist.angular = quadrotor_common::eigenToGeometry(des_yaw_msg);

    inter_distance_error_pub_.publish(msg);
}

Eigen::Vector2d ExpFormationController::computeUav2VelCmd(){
    Eigen::Matrix2d A2, C, D;
    A2.row(0) = (pos_uav2_ - pos_uav1_).transpose();
    A2.row(1) = (pos_uav2_ - pos_uav3_).transpose();

    Eigen::Vector2d b2 = Eigen::Vector2d(inter_distance_error_.x(), inter_distance_error_.z());

    C << coefficient_ / 2.0, 0,
            0, coefficient_ / 4.0;

    D << pos_uav2_.y() - pos_uav3_.y(), -(pos_uav2_.y() - pos_uav1_.y()),
            -(pos_uav2_.x() - pos_uav3_.x()), pos_uav2_.x() - pos_uav1_.x();

    ROS_DEBUG_STREAM("A2: " << A2 << "\n C: " << C << "\n D: " << D << "\n b2" << b2);

    Eigen::Vector2d u2;
    if(abs(A2.determinant()) < 1e-2){
        ready_to_formation_ = false;
        ROS_ERROR("The deter of A2 is nearly zero!!!");
    }
    u2 = - 1.0 / A2.determinant() * C * D * b2;
    return u2;
}

Eigen::Vector2d ExpFormationController::computeUav3VelCmd(){
    Eigen::Matrix2d A3, C, D;
    A3.row(0) = (pos_uav3_ - pos_uav1_).transpose();
    A3.row(1) = (pos_uav3_ - pos_uav2_).transpose();

    Eigen::Vector2d b3 = Eigen::Vector2d(inter_distance_error_.y(), inter_distance_error_.z());

    C << coefficient_ / 2.0, 0,
            0, coefficient_ / 4.0;

    D << pos_uav3_.y() - pos_uav2_.y(), -(pos_uav3_.y() - pos_uav1_.y()),
            -(pos_uav3_.x() - pos_uav2_.x()), pos_uav3_.x() - pos_uav1_.x();

    Eigen::Vector2d u3;
    if(abs(A3.determinant()) < 1e-3){
        ready_to_formation_ = false; 
        ROS_ERROR("The deter of A3 is nearly zero!!!");
    }

    ROS_DEBUG_STREAM("A3: " << A3 << "\n C: " << C << "\n D: " << D << "\n b2" << b3);
    u3 = - 1.0 / A3.determinant() * C * D * b3;
    return u3;
}

void ExpFormationController::computeUav4DesPosition(){
    Eigen::Vector2d center_of_line = (pos_uav2_ + pos_uav3_) / 2.0;
    Eigen::Vector2d perp_of_line;
    perp_of_line << -(pos_uav3_.y() - pos_uav2_.y()),
                    pos_uav3_.x() - pos_uav2_.x();

    if(perp_of_line.norm() != 0){
        perp_of_line = perp_of_line / perp_of_line.norm();
    }
    else{
        ROS_ERROR("The perpendicular vector of line from pos_uav2 to pos_uav3 is zero!!!");
    }
    
    

    uav4_des_pos_ = center_of_line + perp_of_line * (pos_uav3_ - pos_uav2_).norm() * sqrt(3) / 2.0;

}

void ExpFormationController::computeDesYaw(){
    des_yaw_.x() = atan2(pos_uav1_.y() - pos_uav2_.y(), pos_uav1_.x() - pos_uav2_.x());
    des_yaw_.y() = atan2(pos_uav1_.y() - pos_uav3_.y(), pos_uav1_.x() - pos_uav3_.x());
    des_yaw_.z() = atan2(pos_uav1_.y() - pos_uav4_.y(), pos_uav1_.y() - pos_uav4_.x());
}

void ExpFormationController::readyToFormationCallback(const std_msgs::BoolConstPtr & msg){
    ready_to_formation_  = msg->data;
    ready_to_init_position_ = false;
}

void ExpFormationController::uav4ReadyToFlyCallback(const std_msgs::BoolConstPtr & msg){
    uav4_ready_to_fly_ = msg->data;
    computeUav4DesPosition();
    uav2_des_pos_.x() = exp_uav2_.pos_.x();
    uav2_des_pos_.y() = exp_uav2_.pos_.y();
    uav2_des_pos_.z() = uav2_init_pos_.z();

    uav3_des_pos_.x() = exp_uav3_.pos_.x();
    uav3_des_pos_.y() = exp_uav3_.pos_.y();
    uav3_des_pos_.z() = uav3_init_pos_.z();
    ready_to_formation_ = false;
}

void ExpFormationController::readyToYawCallback(const std_msgs::BoolConstPtr & msg){
    ready_to_yaw_ = msg->data;
    computeDesYaw();
    uav4_ready_to_fly_ = false;
}

void ExpFormationController::initializeState(){
    //wait for service of arming and set_mode
    exp_uav2_.arming_client_.waitForExistence();
    exp_uav2_.set_mode_client_.waitForExistence();

    exp_uav3_.arming_client_.waitForExistence();
    exp_uav3_.set_mode_client_.waitForExistence();

    exp_uav4_.arming_client_.waitForExistence();
    exp_uav4_.set_mode_client_.waitForExistence();
    ROS_INFO("The service for arming and set_mode is ready");
    //wait for FCU connection
    while (!exp_uav2_.state_.connected
        || !exp_uav3_.state_.connected
        || !exp_uav4_.state_.connected){
            ros::Rate rate(20.0);
            rate.sleep();
            ROS_INFO("The onboard computer does not connect the fcu!");
            ros::spinOnce();
    }
}

void ExpFormationController::initCenterOfCoordinate(){
    while (exp_uav2_.global_position_.status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX
               || abs(exp_uav2_.global_position_.latitude) < 1.0){
        ROS_WARN("The gps of uav2 is no fix");
        ros::spinOnce();
    }
    ROS_INFO("The gps of uav2 is fix");
    
    geographic_msgs::GeoPointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.position.latitude = exp_uav2_.global_position_.latitude;
    msg.position.longitude = exp_uav2_.global_position_.longitude;
    msg.position.altitude = exp_uav2_.global_position_.altitude;

    exp_uav2_.pubSetGPOrigin(msg);
    exp_uav3_.pubSetGPOrigin(msg);
    exp_uav4_.pubSetGPOrigin(msg);

    ROS_INFO("global_position_origin/latitude: %f, longitude: %f, altitude: %f", 
                                        msg.position.latitude, msg.position.longitude, msg.position.altitude);
}

void ExpFormationController::setMode(){
    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.custom_mode = "OFFBOARD";

    if(exp_uav2_.state_.armed && exp_uav2_.state_.mode != "OFFBOARD"){
        if(exp_uav2_.set_mode_client_.call(set_mode_srv) &&
                set_mode_srv.response.mode_sent){
                        ROS_INFO("uav2 offboard enabled");
        } 
        else{
              ROS_INFO("fail to enable uav2 offboard");
        }
    }
    
    if(exp_uav3_.state_.armed && exp_uav3_.state_.mode != "OFFBOARD"){
        if(exp_uav3_.set_mode_client_.call(set_mode_srv) &&
                set_mode_srv.response.mode_sent){
                        ROS_INFO("uav3 offboard enabled");
        }
        else{
              ROS_INFO("fail to enable uav3 offboard");
        }
    }

    if(exp_uav4_.state_.armed && exp_uav4_.state_.mode != "OFFBOARD"){
        if(exp_uav4_.set_mode_client_.call(set_mode_srv) &&
                set_mode_srv.response.mode_sent){
                        ROS_INFO("uav4 offboard enabled");
        }
        else{
              ROS_INFO("fail to enable uav4 offboard");
        }
    }
}

//latitude weidu longitude jingdu
void ExpFormationController::loadParameter(){
    std::vector<double> pos_uav1;

    pnh_.param<std::vector<double>>("pos_uav1", pos_uav1, 
                    std::vector<double>({-3.0, 4.0}));

    pos_uav1_ = Eigen::Vector2d(pos_uav1[0], pos_uav1[1]);

    std::vector<double> uav2_init_pos;
    std::vector<double> uav3_init_pos;
    std::vector<double> uav4_init_pos;
    std::vector<double> des_inter_distance_square;

    pnh_.param<std::vector<double>>("uav2_init_pos", uav2_init_pos, 
                    std::vector<double>({3.0, -2.0, 2.0}));

    pnh_.param<std::vector<double>>("uav3_init_pos", uav3_init_pos, 
                    std::vector<double>({2.0, 3.0, 2.0}));

    pnh_.param<std::vector<double>>("uav4_init_pos", uav4_init_pos, 
                    std::vector<double>({-3.0, 0.0, 2.0}));                                                                    
    
    while (abs(exp_uav2_.home_height_) < 0.01 
                || abs(exp_uav3_.home_height_) < 0.01
                || abs(exp_uav4_.home_height_) < 0.01 ){
        ROS_WARN("The home height fail to set");
        ros::spinOnce();
    }
    
    uav2_init_pos[2] = uav2_init_pos[2] + exp_uav2_.home_height_;
    uav3_init_pos[2] = uav3_init_pos[2] + exp_uav3_.home_height_;
    uav4_init_pos[2] = uav4_init_pos[2] + exp_uav4_.home_height_;

    uav2_init_pos_ = Eigen::Vector3d(uav2_init_pos[0], uav2_init_pos[1], uav2_init_pos[2]) 
                            + Eigen::Vector3d(pos_uav1_[0], pos_uav1_[1], 0.0);
    uav3_init_pos_ = Eigen::Vector3d(uav3_init_pos[0], uav3_init_pos[1], uav3_init_pos[2]) 
                            + Eigen::Vector3d(pos_uav1_[0], pos_uav1_[1], 0.0);
    uav4_init_pos_ = Eigen::Vector3d(uav4_init_pos[0], uav4_init_pos[1], uav4_init_pos[2]) 
                            + Eigen::Vector3d(pos_uav1_[0], pos_uav1_[1], 0.0);

    pnh_.param<std::vector<double>>("des_inter_distance_square", des_inter_distance_square, 
                    std::vector<double>({4.0, 4.0, 12.0}));       

    for (size_t i = 0; i < 3; i++){
        des_inter_distance_[i] = sqrt(des_inter_distance_square[i]);
    }
    
    pnh_.param<double>("gain_m", gain_m_, 0.01);
    pnh_.param<double>("gain_n", gain_n_, 0.04);
    pnh_.param<double>("threshold", threshold_, 5.0);
}

double ExpFormationController::pow2(double x){
    return x * x;
}

