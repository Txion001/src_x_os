/*
 * Designed by Tom Xiong
 * 
 * OBJECTIVE:
 * TO ENABLE INTERACTION WITH THE HABITAT BUILDING
 * 
 * OWNERSHIP:
 * TEAM 1476-"XION SYSTEMS"
 * 
 * 
 */
#ifndef _HABITAT_DRIVER_HPP
#define _HABITAT_DRIVER_HPP

#include "ros/ros.h"
#include "math.h"
#include "map_scan/map_scan.hpp"
#include "gait_generation/gait_generation.hpp"

class Habitat_Driver
{
  private:
    ros::NodeHandle habNodeHandle;
    
    Trajectory_Generation trajectory_generation;
    Gait_Generation gait_generation;
    
    const bool LEFT = 0;
    const bool RIGHT = 1;
    const std::string WORLD = "world";
    const std::string RIGHT_FOOT_ANKLE = "rightFoot";
    const std::string LEFT_FOOT_ANKLE = "leftFoot";
    const std::string RIGHT_FOOT_SOLE = "rightCOP_Frame";
    const std::string LEFT_FOOT_SOLE = "leftCOP_Frame";
    const std::string RIGHT_WRIST = "rightWristRollLink";
    const std::string LEFT_WRIST = "leftWristRollLink";
    const std::string PELVIS = "pelvis";
    const std::string TORSO = "torso";
    
  public:
    Habitat_Driver();
    void
    climbStairs(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    walkToDoor(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    alignToDoor(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    openDoor();
    std::vector<float>
    openDoorTrajectory(bool robot_side_arm, int step_num);
    void
    enterHab();
    std::vector<float>
    getArmAtStair(bool robot_side);
};

#endif
