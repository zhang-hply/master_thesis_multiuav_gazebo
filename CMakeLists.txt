cmake_minimum_required(VERSION 2.8.3)
project("multiuav_formation")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    quadrotor_msgs
    quadrotor_common
    polynomial_trajectories
    xmlrpcpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    cv_bridge
)


find_package(Eigen REQUIRED)

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


