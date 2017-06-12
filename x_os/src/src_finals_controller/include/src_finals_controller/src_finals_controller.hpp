/*
 * Designed by Tom Xiong
 * 
 * OBJECTIVE:
 * TO ORGANIZE AND CONTROL OPERATION OF TASKS
 * 
 * OWNERSHIP:
 * TEAM 1476-"XION SYSTEMS"
 * 
 * 
 */
#ifndef _SRC_FINALS_CONTROLLER_HPP
#define _SRC_FINALS_CONTROLLER_HPP

#include "ros/ros.h"
#include "map_scan/map_scan.hpp"
#include "gait_generation/gait_generation.hpp"

class Src_Finals_Controller
{
  private:
    ros::NodeHandle srcNodeHandle;
    
    ros::Publisher scan_output;
    
    Trajectory_Generation trajectory_generation;
    Gait_Generation gait_generation;
    Map_Scan map_scan;
    
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
    
    pcl::PointCloud<pcl::PointXYZRGB> map;
    
    void
    exploreMap(tf2_ros::Buffer* tfBuffer);
    void
    exploreMap(tf2_ros::Buffer* tfBuffer, float distance_to_explore, bool arms_disabled);
    void
    executeTaskOne(tf2_ros::Buffer* tfBuffer);
    void
    executeTaskTwo(tf2_ros::Buffer* tfBuffer);
    void
    executeTaskThree(tf2_ros::Buffer* tfBuffer);
    void
    executeAllTasks(tf2_ros::Buffer* tfBuffer, ros::Publisher* statusPublisher);
  
  public:
    Src_Finals_Controller();
    
    void
    runController(tf2_ros::Buffer* tfBuffer, int task_id, ros::Publisher* statusPublisher);
};

#endif
