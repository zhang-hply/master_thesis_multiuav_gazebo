#include "exp_uav.h"

ExpUAV::ExpUAV(const ros::NodeHandle & nh, const ros::NodeHandle & pnh)\
    :nh_(nh),
    pnh_(pnh){
        state_sub_ = nh_.subscribe("mavros/state",  10, 
                            &ExpUAV::stateCallback, this);

        pose_sub_ = nh_.subscribe("mavros/local_position/pose", 10,
                            &ExpUAV::poseCallback, this);

        gp_origin_sub_ = nh_.subscribe("mavros/global_position/gp_origin", 10,
                            &ExpUAV::gpOriginCallback, this);
        
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
    yaw_msg.vector.x = yaw_;
    yaw_pub_.publish(yaw_msg);
}

void ExpUAV::gpOriginCallback(const geographic_msgs::GeoPointStampedConstPtr & msg){
    gp_origin_ = *msg;
}

void ExpUAV::echoGPOrigin(){
    ROS_INFO("the latitude is %f, longitude is %f, the altitude is %f", 
                    gp_origin_.position.latitude,
                    gp_origin_.position.longitude,
                    gp_origin_.position.altitude);
}


void ExpUAV::pubYawCmdVel(const double des_yaw){
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    double gain = 1.1;
    double yaw_rate = gain * (des_yaw - yaw_);
    yaw_rate = yaw_rate < 0.5 ? (yaw_rate  > -0.5 ? yaw_rate : -0.5) : 0.5;
    msg.twist.angular.z = yaw_rate;
    local_cmd_vel_pub_.publish(msg);
}






