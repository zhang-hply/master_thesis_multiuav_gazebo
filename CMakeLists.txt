cmake_minimum_required(VERSION 2.8.3)
project("multiuav_formation")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    quadrotor_msgs
    quadrotor_common
    xmlrpcpp
    nav_msgs
    geometry_msgs
    geographic_msgs
    mavros_msgs
    sensor_msgs
    cv_bridge
)


find_package(Eigen3 REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIR}
    include
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs
)


add_library(formation_controller src/formation_controller.cpp)

add_executable(formation_controller_node src/formation_controller_node.cpp)
target_link_libraries(formation_controller_node 
    formation_controller
    ${catkin_LIBRARIES} 
    ${Eigen_LIBRARIES}
)

add_library(exp_uav src/exp_uav.cpp)
add_library(exp_formation_controller src/exp_formation_controller.cpp)

add_executable(exp_formation_controller_node src/exp_formation_controller_node.cpp)
target_link_libraries(exp_formation_controller_node 
    exp_formation_controller
    exp_uav
    ${catkin_LIBRARIES} 
    ${Eigen_LIBRARIES}
)


