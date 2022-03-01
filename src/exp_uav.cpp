#include "exp_uav.h"

ExpUAV::ExpUAV(const ros::NodeHandle & nh, const ros::NodeHandle & pnh)\
    :nh_(nh),
    pnh_(pnh){
        state_sub_ = nh_.subscribe("mavros/state",  10, 
                            &ExpUAV::stateCallback, this);

        pose_sub_ = nh_.subscribe("mavros/local_position/pose", 10,
                            &ExpUAV::poseCallback, this);
        
        local_cmd_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
                            "mavros/setpoint_velocity/cmd_vel", 10);

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

void ExpUAV::pubLocalCmdVel(const Eigen::Vector2d & u){
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = u[0];
    msg.twist.linear.y = u[1];
    local_cmd_vel_pub_.publish(msg);
}

void ExpUAV::pubSetGPOrigin(const geographic_msgs::GeoPointStamped & msg){
    set_gp_origin_pub_.publish(msg);
}

void ExpUAV::poseCallback(const geometry_msgs::PoseStampedConstPtr & msg){
    pos_[0] = msg->pose.position.x;
    pos_[1] = msg->pose.position.y;

    yaw_ =  (quadrotor_common::quaternionToEulerAnglesZYX(
        quadrotor_common::geometryToEigen(msg->pose.orientation))).z();
}

void ExpUAV::pubYawCmdVel(const double des_yaw){
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    double gain = 1.1;
    msg.twist.angular.z = gain * (des_yaw - yaw_);
    local_cmd_vel_pub_.publish(msg);
}






