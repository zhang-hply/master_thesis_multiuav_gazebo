#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "exp_uav.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <quadrotor_common/geometry_eigen_conversions.h>

class ExpFormationController
{
public:
    ExpFormationController(const ros::NodeHandle & nh, const ros::NodeHandle & pnh);
    ExpFormationController():
        ExpFormationController(ros::NodeHandle(), ros::NodeHandle("~")){}
    ~ExpFormationController();


    void mainloop(const ros::TimerEvent & time);

private:
    ros::NodeHandle nh_, pnh_;
    ExpUAV exp_uav2_, exp_uav3_, exp_uav4_;
    std::vector<double> gp_origin_;

    Eigen::Vector3d uav2_init_pos_;
    Eigen::Vector3d uav3_init_pos_;
    Eigen::Vector3d uav4_init_pos_;

    Eigen::Vector3d uav2_des_pos_;
    Eigen::Vector3d uav3_des_pos_;

    ros::Timer main_loop_timer_;

    ros::Subscriber ready_to_formation_sub_;
    ros::Subscriber uav4_ready_to_fly_sub_;
    ros::Subscriber ready_to_yaw_sub_;
    ros::Publisher inter_distance_error_pub_;

    bool ready_to_init_position_;
    bool ready_to_formation_;
    bool uav4_ready_to_fly_;
    bool ready_to_yaw_;

    Eigen::Vector2d pos_uav1_;
    Eigen::Vector2d pos_uav2_;
    Eigen::Vector2d pos_uav3_;
    Eigen::Vector2d pos_uav4_;
    Eigen::Vector2d des_pos_uav4_;
    Eigen::Vector3d yaw_;
    Eigen::Vector3d des_yaw_;

    double coefficient_;
    double gain_m_, gain_n_;
    double threshold_;

    Eigen::Vector3d inter_distance_error_;
    Eigen::Vector3d des_inter_distance_;

    void readyToFormationCallback(const std_msgs::BoolConstPtr & msg);
    void uav4ReadyToFlyCallback(const std_msgs::BoolConstPtr & msg);
    void readyToYawCallback(const std_msgs::BoolConstPtr & msg);

    void computeInterDistance();
    Eigen::Vector2d computeUav2VelCmd();
    Eigen::Vector2d computeUav3VelCmd();
    void computeUav4DesPosition();
    void computeDesYaw();
    double pow2(double x);

    void loadParameter();
    void initializeState();
    void setMode();

    Eigen::Vector3d vector2EigenVector3d(const std::vector<double> & v);
};


