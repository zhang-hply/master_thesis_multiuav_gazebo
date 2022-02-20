#include <iostream>
#include <ros/ros.h>
#include "formation_controller.h"


 int main(int argc, char ** argv){
     ros::init(argc, argv, "formation_controller_node");
    FormationController formation_controller;
    ros::spin();
    return 0;
 }