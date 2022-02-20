#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

class FormationController
{
public:
    FormationController(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    FormationController():FormationController(ros::NodeHandle(), ros::NodeHandle("~")){};
    ~FormationController();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber odometry_sub_uav2_;
    ros::Subscriber odometry_sub_uav3_;
    ros::Subscriber odometry_sub_uav4_;
    ros::Subscriber ready_to_formation_sub_;
    ros::Subscriber uav4_ready_to_fly_sub_;
    ros::Subscriber ready_to_yaw_sub_;

    ros::Publisher vel_cmd_pub_uav2_;
    ros::Publisher vel_cmd_pub_uav3_;
    ros::Publisher vel_cmd_pub_uav4_;
    ros::Publisher inter_distance_error_pub_;
    ros::Publisher yaw_pub_;

    ros::Timer main_loop_timer_;

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

    void mainloop(const ros::TimerEvent & time);
    
    void uav2OdometryCallback(const nav_msgs::OdometryConstPtr & msg);
    void uav3OdometryCallback(const nav_msgs::OdometryConstPtr & msg);
    void uav4OdometryCallback(const nav_msgs::OdometryConstPtr & msg);
    void readyToFormationCallback(const std_msgs::BoolConstPtr & msg);
    void uav4ReadyToFlyCallback(const std_msgs::BoolConstPtr & msg);
    void readyToYawCallback(const std_msgs::BoolConstPtr & msg);

    void computeInterDistance();
    Eigen::Vector2d computeUav2VelCmd();
    Eigen::Vector2d computeUav3VelCmd();
    void publishVelCmd(ros::Publisher & pub, Eigen::Vector2d & u);
    void publishYawCmd(ros::Publisher & pub, int  id);

    void loadparameter();
    void computeUav4DesPosition();
    void computeDesYaw();
    double pow2(double x);
};


