/*
 * Designed by Tom Xiong
 * 
 * OBJECTIVE:
 * TO ENABLE INTERACTION WITH THE SATELLITE DISH HANDLES
 * UTILIZATION OF ALL TRAJECTORIES
 * 
 * OWNERSHIP:
 * TEAM 1476-"XION SYSTEMS"
 * 
 * TODO: Create scanning functions for task 1, 2, 3
 * 
 */
#ifndef _SATELLITE_DISH_DRIVER_HPP
#define _SATELLITE_DISH_DRIVER_HPP

#include "ros/ros.h"
#include "map_scan/map_scan.hpp"
#include "gait_generation/gait_generation.hpp"
#include "srcsim/Satellite.h"

class Satellite_Dish_Driver
{
  private:
    ros::NodeHandle dishNodeHandle;
    
    ros::Publisher stepListPublisher;
    ros::Publisher trajectoryListPublisher;
    ros::Publisher trajectoryNeckPublisher;
    ros::Publisher trajectoryRightHandPublisher;
    ros::Publisher trajectoryLeftHandPublisher;
    ros::Subscriber dish_angle_sub;
    ros::CallbackQueue dish_angle_sub_callback;
    
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
    
    float target_pitch;
    float target_yaw;
    float current_pitch;
    float current_yaw;
    bool pitch_completed = false;
    bool yaw_completed = false;
    
    void
    getDishAngle(const srcsim::Satellite& msg);
    std::vector<float>
    grabTableTrajectory(bool robot_side_arm, int step_num);
    std::vector<float>
    moveHandlesTrajectory(bool robot_side_arm, int step_num);
    std::vector<float>
    getArmZeroAngVel();
  
  public:
    Satellite_Dish_Driver();
    /*
     * Performs operation of satellite dish handles
     * RUN TASK ONE DRIVER: Gets Valkyrie into position to control the panel
     * ALIGN DISH: fixes the angle and distance from panel
     * CENTER BETWEEN ARMS: Checks for x direction 
     */
    void
    alignDish();
    void
    grabTableBottom();
    void
    grabTableSide();
    void
    checkTableSide();
    void
    centerBetweenArms();
    void
    letGoTable();
    void
    alignArms();
    void
    startPitchController();
    void
    increasePitch();
    void
    decreasePitch();
    void
    startYawController();
    void
    increaseYaw();
    void
    decreaseYaw();
    void
    resetStance(std::vector<float> next_point);
};

#endif
