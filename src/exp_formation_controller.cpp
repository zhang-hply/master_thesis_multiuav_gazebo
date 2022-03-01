#include "exp_formation_controller.h"

ExpFormationController::ExpFormationController(
    const ros::NodeHandle & nh, const ros::NodeHandle & pnh)
    :nh_(nh),
    pnh_(pnh),
    exp_uav2(ros::NodeHandle("uav2"), pnh_),
    exp_uav3(ros::NodeHandle("uav3"), pnh_),
    exp_uav4(ros::NodeHandle("uav4"), pnh_){
        loadParameter();
        ROS_INFO("global_position_origin: %f, %f, %f", gp_origin_[0], gp_origin_[1], gp_origin_[2]);
        initializeState();
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
    pos_uav2_ = exp_uav2.pos_;
    pos_uav3_ = exp_uav2.pos_;
    pos_uav4_ = exp_uav2.pos_;

    if(ready_to_formation_){
        computeInterDistance();
        Eigen::Vector2d u2 = computeUav2VelCmd();
        Eigen::Vector2d u3 = computeUav3VelCmd();
        exp_uav2.pubLocalCmdVel(u2);
        exp_uav3.pubLocalCmdVel(u3);
    }

    if(uav4_ready_to_fly_){
        ROS_INFO_ONCE("Enter the uav4");
        Eigen::Vector2d u4;
        u4[0] = 0.4 * (des_pos_uav4_.x() - pos_uav4_.x());
        u4[1] = 0.4 * (des_pos_uav4_.y() - pos_uav4_.y());

        exp_uav4.pubLocalCmdVel(u4);
    }

    if(ready_to_yaw_){
        exp_uav2.pubYawCmdVel(des_yaw_.x());
        exp_uav3.pubYawCmdVel(des_yaw_.y());
        exp_uav4.pubYawCmdVel(des_yaw_.z());
    }
}

void ExpFormationController::initializeState(){
    //wait for service of arming and set_mode
    exp_uav2.arming_client_.waitForExistence();
    exp_uav2.set_mode_client_.waitForExistence();

    exp_uav3.arming_client_.waitForExistence();
    exp_uav3.set_mode_client_.waitForExistence();

    exp_uav4.arming_client_.waitForExistence();
    exp_uav4.set_mode_client_.waitForExistence();
    ROS_INFO("The service for arming and set_mode is ready");
    //wait for FCU connection
    while (ros::ok() && exp_uav2.state_.connected
        && exp_uav3.state_.connected
        && exp_uav4.state_.connected){
            ros::Rate rate(20.0);
            rate.sleep();
    }

    geographic_msgs::GeoPointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.position.latitude = gp_origin_[0];
    msg.position.longitude = gp_origin_[1];
    msg.position.altitude = gp_origin_[2];

    exp_uav2.pubSetGPOrigin(msg);
    exp_uav3.pubSetGPOrigin(msg);
    exp_uav4.pubSetGPOrigin(msg);
}
//latitude weidu longitude jingdu
void ExpFormationController::loadParameter(){
    pnh_.param<std::vector<double>>("gp_origin", gp_origin_, 
                    std::vector<double>({31.81746431549553,
                                                                117.13249134391783,
                                                                33.0}));
}

void ExpFormationController::setMode(){
    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.custom_mode = "OFFBOARD";

    if(exp_uav2.state_.armed && exp_uav2.state_.mode != "OFFBOARD"){
        while(!(exp_uav2.set_mode_client_.call(set_mode_srv) 
                    && set_mode_srv.response.mode_sent)){
                        ROS_INFO("set uav2 mode to offboard unsuccessfully");
                    }

        ROS_INFO("uav2 offboard enabled");
    }
    

    if(exp_uav3.state_.armed && exp_uav3.state_.mode != "OFFBOARD"){
        while(!(exp_uav3.set_mode_client_.call(set_mode_srv) 
                    && set_mode_srv.response.mode_sent)){
                        ROS_INFO("set uav3 mode to offboard unsuccessfully");
                    }

        ROS_INFO("uav3 offboard enabled");
    }

    if(exp_uav4.state_.armed && exp_uav4.state_.mode != "OFFBOARD"){
        while(!(exp_uav4.set_mode_client_.call(set_mode_srv) 
                    && set_mode_srv.response.mode_sent)){
                        ROS_INFO("set uav4 mode to offboard unsuccessfully");
                    }

        ROS_INFO("uav4 offboard enabled");
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
        ROS_INFO_ONCE("the coefficient is 0.4");
        coefficient_ = gain_m_;
    }else{
        ROS_INFO_ONCE("the coefficient is 1.0");
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

    Eigen::Vector2d u2;
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
    u3 = - 1.0 / A3.determinant() * C * D * b3;
    return u3;
}

void ExpFormationController::readyToFormationCallback(const std_msgs::BoolConstPtr & msg){
    ready_to_formation_  = msg->data;
}

void ExpFormationController::uav4ReadyToFlyCallback(const std_msgs::BoolConstPtr & msg){
    uav4_ready_to_fly_ = msg->data;
    computeUav4DesPosition();
    ready_to_formation_ = false;
}

void ExpFormationController::readyToYawCallback(const std_msgs::BoolConstPtr & msg){
    ready_to_yaw_ = msg->data;
    computeDesYaw();
    uav4_ready_to_fly_ = false;
}

void ExpFormationController::computeUav4DesPosition(){
    pos_uav2_ = pos_uav2_ - pos_uav1_;
    pos_uav3_ = pos_uav3_ - pos_uav1_;

    double x = (pow2(pos_uav3_.x()) + pow2(pos_uav3_.y())) /
                                            (1 + pow2(pos_uav3_.x() - pos_uav2_.x()) / 
                                            pow2(pos_uav3_.y() - pos_uav2_.y()));

    des_pos_uav4_.x()  = -sqrt(x);
    des_pos_uav4_.y() = -(pos_uav3_.x() - pos_uav2_.x())/
                                            (pos_uav3_.y() - pos_uav2_.y()) * des_pos_uav4_.x();

    des_pos_uav4_ = des_pos_uav4_ + pos_uav1_;
    ROS_INFO_ONCE("x:%f, y:%f", des_pos_uav4_.x(), des_pos_uav4_.y());
}

void ExpFormationController::computeDesYaw(){
    des_yaw_.x() = atan2(pos_uav1_.y() - pos_uav2_.y(), pos_uav1_.x() - pos_uav2_.x());
    des_yaw_.y() = atan2(pos_uav1_.y() - pos_uav3_.y(), pos_uav1_.x() - pos_uav3_.x());
    des_yaw_.z() = atan2(pos_uav1_.y() - pos_uav4_.y(), pos_uav1_.y() - pos_uav4_.x());
}

double ExpFormationController::pow2(double x){
    return x * x;
}