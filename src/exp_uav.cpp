#include "exp_uav.h"

ExpUAV::ExpUAV(const ros::NodeHandle & nh, const ros::NodeHandle & pnh)\
    :nh_(nh),
    pnh_(pnh){
        state_sub_ = nh_.subscribe("mavros/state",  10, 
                            &ExpUAV::stateCallback, this);

        pose_sub_ = nh_.subscribe("mavros/local_position/pose", 10,
                            &ExpUAV::poseCallback, this);

        global_position_sub_ = nh_.subscribe("mavros/global_position/global", 10,
                            &ExpUAV::globalPositionCallback, this);
        
        home_position_sub_ = nh_.subscribe("mavros/home_position/home", 10,
                            &ExpUAV::homePositionCallback, this);

        local_cmd_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
                            "mavros/setpoint_velocity/cmd_vel", 10);

        local_position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
                            "mavros/setpoint_position/local", 10);

        yaw_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
                            "yaw", 10);


        set_gp_origin_pub_ = nh_.advertise<geographic_msgs::GeoPointStamped>(
                            "mavros/global_position/set_gp_origin", 10);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
                            "mavros/cmd/arming");   

        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
                            "mavros/set_mode");
}

ExpUAV::~ExpUAV(){}

void ExpUAV::stateCallback(const mavros_msgs::StateConstPtr & msg){
    state_ = *msg;
}

void ExpUAV::pubLocalCmdVel(Eigen::Vector2d & u, const double & des_height){
    u = u.cwiseMin(Eigen::Vector2d(1.0, 1.0));
    u = u.cwiseMax(Eigen::Vector2d(-1.0, -1.0));

    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = u[0];
    msg.twist.linear.y = u[1];
    double uz = - (current_height_ - des_height);
    uz = uz < 0.5 ? (uz  > -0.5 ? uz : -0.5) : 0.5;
    msg.twist.linear.z = uz;
    
    local_cmd_vel_pub_.publish(msg);
}

void ExpUAV::pubLocalPosition(Eigen::Vector3d & position){
    if(position.maxCoeff() > 100 || position.minCoeff() < -100){
        ROS_ERROR("The position command is unavaliable");
        exit(4);
    }

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = position.x();
    msg.pose.position.y = position.y();
    msg.pose.position.z = position.z();
    local_position_pub_.publish(msg);
}

void ExpUAV::pubSetGPOrigin(const geographic_msgs::GeoPointStamped & msg){
    set_gp_origin_pub_.publish(msg);
}

void ExpUAV::poseCallback(const geometry_msgs::PoseStampedConstPtr & msg){
    pos_[0] = msg->pose.position.x;
    pos_[1] = msg->pose.position.y;
    current_height_ = msg->pose.position.z;
    yaw_ =  (quadrotor_common::quaternionToEulerAnglesZYX(
        quadrotor_common::geometryToEigen(msg->pose.orientation))).z();
    
    geometry_msgs::Vector3Stamped yaw_msg;
    yaw_msg.header.stamp = ros::Time::now();
    yaw_msg.vector.x = yaw_ / M_PI * 180.0;
    yaw_msg.vector.y = des_yaw_ /M_PI * 180.0;
    yaw_pub_.publish(yaw_msg);
}

void ExpUAV::globalPositionCallback(const sensor_msgs::NavSatFixConstPtr & msg){
    global_position_ = *msg;
}

void ExpUAV::pubYawCmdVel(const double des_yaw){
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    double gain = 1.1;
    double yaw_rate = gain * (des_yaw - yaw_);
    ROS_DEBUG("%s, yaw_rate: %f", nh_.getNamespace().c_str(), yaw_rate);
    yaw_rate = yaw_rate < 0.5 ? (yaw_rate  > -0.5 ? yaw_rate : -0.5) : 0.5;
    msg.twist.angular.z = yaw_rate;
    local_cmd_vel_pub_.publish(msg);
    des_yaw_ = des_yaw;
    
}

void ExpUAV::homePositionCallback(const mavros_msgs::HomePositionConstPtr & msg){
    home_height_ = (*msg).position.z;
}

//直接使用mavros/setpoint_position/local发送yaw指令
void ExpUAV::pubYawCmdPos(const Eigen::Vector3d & position, const double des_yaw){
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = position.x();
    msg.pose.position.y = position.y();
    msg.pose.position.z = position.z();
    msg.pose.orientation = quadrotor_common::eigenToGeometry(
                            quadrotor_common::eulerAnglesZYXToQuaternion(
                            Eigen::Vector3d(0.0, 0.0, des_yaw)));
    ROS_DEBUG_STREAM(nh_.getNamespace() << "yawCmdPos: " << msg.pose.position << msg.pose.orientation);
    local_position_pub_.publish(msg);
    des_yaw_ = des_yaw;
}



