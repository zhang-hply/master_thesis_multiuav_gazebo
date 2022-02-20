#include "formation_controller.h"

FormationController::FormationController(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    :nh_(nh),
    pnh_(pnh),
    pos_uav1_(Eigen::Vector2d(0,0)),
    ready_to_formation_(false),
    uav4_ready_to_fly_(false)
{
      loadparameter();
    odometry_sub_uav2_ = nh_.subscribe("/uav2/ground_truth/odometry", 1,
        &FormationController::uav2OdometryCallback, this);
    
    odometry_sub_uav3_ = nh_.subscribe("/uav3/ground_truth/odometry",1,
        &FormationController::uav3OdometryCallback, this);

    odometry_sub_uav4_ = nh_.subscribe("/uav4/ground_truth/odometry",1,
        &FormationController::uav4OdometryCallback, this);


    ready_to_formation_sub_ = nh_.subscribe("/ready_to_formation", 1,
        &FormationController::readyToFormationCallback, this);
    
    ready_to_formation_sub_ = nh_.subscribe("/ready_to_formation", 1,
        &FormationController::readyToFormationCallback, this);

    uav4_ready_to_fly_sub_ = nh_.subscribe("/uav4_ready_to_fly", 1,
        &FormationController::uav4ReadyToFlyCallback, this);

    ready_to_yaw_sub_ = nh_.subscribe("/ready_to_yaw", 1, 
        &FormationController::readyToYawCallback, this);

    vel_cmd_pub_uav2_ = nh_.advertise<geometry_msgs::TwistStamped>(
        "/uav2/autopilot/velocity_command", 1
    );

    vel_cmd_pub_uav3_ = nh_.advertise<geometry_msgs::TwistStamped>(
        "/uav3/autopilot/velocity_command", 1
    );

    vel_cmd_pub_uav4_ = nh_.advertise<geometry_msgs::TwistStamped>(
        "/uav4/autopilot/velocity_command", 1
    );

    inter_distance_error_pub_ = nh_.advertise<nav_msgs::Odometry>(
        "/inter_distance", 1
    );

    yaw_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/yaw", 1
    );

    main_loop_timer_ = nh_.createTimer(ros::Duration(0.02), 
        &FormationController::mainloop, this);
}

FormationController::~FormationController(){}

void FormationController::mainloop(const ros::TimerEvent & time){
    if(ready_to_formation_){
        computeInterDistance();
        Eigen::Vector2d u2 = computeUav2VelCmd();
        Eigen::Vector2d u3 = computeUav3VelCmd();
        publishVelCmd(vel_cmd_pub_uav2_, u2);
        publishVelCmd(vel_cmd_pub_uav3_, u3);
    }

    if(uav4_ready_to_fly_){
        ROS_INFO_ONCE("Enter the uav4");
        geometry_msgs::TwistStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.twist.linear.x = 0.4 * (des_pos_uav4_.x() - pos_uav4_.x());
        msg.twist.linear.y = 0.4 * (des_pos_uav4_.y() - pos_uav4_.y());

        vel_cmd_pub_uav4_.publish(msg);
    }

    if(ready_to_yaw_){
        // ROS_INFO("des_yaw/x:%f, y: %f, z: %f", des_yaw_.x(), des_yaw_.y(), des_yaw_.z());
        publishYawCmd(vel_cmd_pub_uav2_, 0);
        publishYawCmd(vel_cmd_pub_uav3_, 1);
        publishYawCmd(vel_cmd_pub_uav4_, 2);
    }

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position = quadrotor_common::vectorToPoint(
        quadrotor_common::eigenToGeometry(yaw_));

    yaw_pub_.publish(msg);

}

void FormationController::uav2OdometryCallback(
    const nav_msgs::OdometryConstPtr & msg){
    geometry_msgs::Point pos = msg->pose.pose.position;
    pos_uav2_.x() = pos.x;
    pos_uav2_.y() = pos.y;
    
    yaw_.x() = (quadrotor_common::quaternionToEulerAnglesZYX(
        quadrotor_common::geometryToEigen(msg->pose.pose.orientation))).z();
}

void FormationController::uav3OdometryCallback(
    const nav_msgs::OdometryConstPtr & msg){
    geometry_msgs::Point pos =   msg->pose.pose.position;
    pos_uav3_.x() = pos.x;
    pos_uav3_.y() = pos.y;

    yaw_.y() = (quadrotor_common::quaternionToEulerAnglesZYX(
        quadrotor_common::geometryToEigen(msg->pose.pose.orientation))).z();
}

void FormationController::uav4OdometryCallback(
    const nav_msgs::OdometryConstPtr & msg){
    geometry_msgs::Point pos = msg->pose.pose.position;
    pos_uav4_.x() = pos.x;
    pos_uav4_.y() = pos.y;

    yaw_.z() = (quadrotor_common::quaternionToEulerAnglesZYX(
        quadrotor_common::geometryToEigen(msg->pose.pose.orientation))).z();
}

void FormationController::readyToFormationCallback(const std_msgs::BoolConstPtr & msg){
    ready_to_formation_  = msg->data;
}

void FormationController::uav4ReadyToFlyCallback(const std_msgs::BoolConstPtr & msg){
    uav4_ready_to_fly_ = msg->data;
    computeUav4DesPosition();
    ready_to_formation_ = false;
}

void FormationController::readyToYawCallback(const std_msgs::BoolConstPtr & msg){
    ready_to_yaw_ = msg->data;
    computeDesYaw();
    uav4_ready_to_fly_ = false;
}

void FormationController::computeInterDistance(){
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

Eigen::Vector2d FormationController::computeUav2VelCmd(){
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

Eigen::Vector2d FormationController::computeUav3VelCmd(){
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

void FormationController::publishVelCmd(ros::Publisher & pub, Eigen::Vector2d & u){
    geometry_msgs::TwistStamped msg;
    msg.header.frame_id="world";
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = u.x();
    msg.twist.linear.y = u.y();
    
    pub.publish(msg);
}

void FormationController::publishYawCmd(ros::Publisher & pub, int  id){
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    // ROS_INFO("yaw/x:%f, y:%f, z:%f", yaw_.x(), yaw_.y(), yaw_.z());
    double gain = 1.1;
    // if(id == 0)
    //     gain = 1.0;
    // else if( id == 1)
    //     gain = 1.0;
    // else if( id == 2)
    //     gain = 1.1;
    
    msg.twist.angular.z = gain * (des_yaw_[id] - yaw_[id]);
    

    pub.publish(msg);
}

void FormationController::loadparameter(){
    gain_m_ = 0.3;
    gain_n_ = 0.6;
    threshold_ = 5.0;

    des_inter_distance_ = Eigen::Vector3d(3.0, 3.0, sqrt(27.0));
}

void FormationController::computeUav4DesPosition(){
    double x = (pow2(pos_uav3_.x()) + pow2(pos_uav3_.y())) /
                                            (1 + pow2(pos_uav3_.x() - pos_uav2_.x()) / 
                                            pow2(pos_uav3_.y() - pos_uav2_.y()));

    des_pos_uav4_.x()  = -sqrt(x);
    des_pos_uav4_.y() = -(pos_uav3_.x() - pos_uav2_.x())/
                                            (pos_uav3_.y() - pos_uav2_.y()) * des_pos_uav4_.x();

    ROS_INFO_ONCE("x:%f, y:%f", des_pos_uav4_.x(), des_pos_uav4_.y());
}

void FormationController::computeDesYaw(){
    des_yaw_.x() = atan2(-pos_uav2_.y(), -pos_uav2_.x());
    des_yaw_.y() = atan2(-pos_uav3_.y(), -pos_uav3_.x());
    des_yaw_.z() = atan2(-pos_uav4_.y(), -pos_uav4_.x());
}

double FormationController::pow2(double x){
    return x * x;
}