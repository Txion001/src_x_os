/*
 * Designed by Tom Xiong
 * 
 * OBJECTIVE:
 * TO PROVIDE DYNAMIC CONTROL OF VALKYRIE'S WALKING THROUGH
 * UTILIZATION OF THE WHOLE BODY/FOOTSTEP CONTROLLERS
 * 
 * OWNERSHIP:
 * TEAM 1476-"XION SYSTEMS"
 * 
 */
#ifndef _GAIT_GENERATION_HPP
#define _GAIT_GENERATION_HPP

#include "ros/ros.h"
#include "trajectory_generation/trajectory_generation.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"
#include "math.h"

struct full_msg
{
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::NeckTrajectoryRosMessage trajectory_neck_msg;
  std_msgs::Float64MultiArray trajectory_finger_left_msg;
  std_msgs::Float64MultiArray trajectory_finger_right_msg;
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
};
struct x_vector
{
  float x;
  float y;
  float z;
};

class Gait_Generation
{
  private :
    Trajectory_Generation trajectory_generation;
    
    const float DEFAULT_STEP_DISTANCE = 0.46;
    const float DEFAULT_SIDE_STEP_DISTANCE = 0.36;
    const float DEFAULT_STEP_SEPERATION = 0.26;
    const float DEFAULT_STEP_HEIGHT = -0.008;
    
    const float DEFAULT_SWING_TIME = 0.60;
    const float DEFAULT_TRANSFER_TIME = 0.38;
    const float FINAL_TRANSFER_TIME = 0.20;
    
    
  public :
    const bool LEFT = 0;
    const bool RIGHT = 1;
    const std::string WORLD = "world";
    const std::string RIGHT_FOOT_ANKLE = "rightFoot";
    const std::string LEFT_FOOT_ANKLE = "leftFoot";
    const std::string RIGHT_FOOT_SOLE = "rightCOP_Frame";
    const std::string LEFT_FOOT_SOLE = "leftCOP_Frame";
    const std::string PELVIS = "pelvis";
    const std::string TORSO = "torso";
    
    Gait_Generation();
    full_msg
    createWakeUp();
    full_msg
    createStanceReset();
    
    std::vector<float>
    getArmSwing(bool robot_side_arm, bool robot_side_step, float step_distance_x, float step_distance_y);
    std::vector<float>
    getArmAtRest(bool robot_side);
    std::vector<float>
    getArmZeroAngVel();
    
    std::vector<float>
    getChestSwing(bool robot_side_step, float step_distance_x, float step_distance_y);
    std::vector<float>
    getChestSwing(bool robot_side_step, float step_distance_x, float step_distance_y, float heading);
    std::vector<float>
    getChestZeroPos();
    std::vector<float>
    getChestZeroAngVel();
    
    // Override based gait generators, designed for single movement
    ihmc_msgs::FootstepDataListRosMessage
    createLowerBodyRotateGaitOverride(float heading);
    ihmc_msgs::FootstepDataListRosMessage
    createLowerBodyLinearGaitOverride(x_vector destination, float heading);
    ihmc_msgs::FootstepDataListRosMessage
    createLowerBodyLinearGaitStairOverride(x_vector destination, float heading, float step_distance);
    ihmc_msgs::WholeBodyTrajectoryRosMessage
    createUpperBodyGait(ihmc_msgs::FootstepDataListRosMessage step_list);
    ihmc_msgs::FootstepDataListRosMessage
    createLowerBodyArcGaitOverride(bool turn, float arc_length, float radian);
    
    // Queued based gait generators, designed for queueing
    ihmc_msgs::FootstepDataListRosMessage
    createLowerBodyRotateGaitQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, float heading);
    ihmc_msgs::FootstepDataListRosMessage
    createLowerBodyLinearGaitQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, x_vector destination, float heading);
    ihmc_msgs::FootstepDataListRosMessage
    createLowerBodyLinearGaitOffsetQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, x_vector destination, float heading);
    ihmc_msgs::FootstepDataListRosMessage
    createLowerBodyLinearGaitOffsetQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, x_vector destination, float heading, float step_distance);
    ihmc_msgs::FootstepDataListRosMessage
    createLowerBodyArcGaitQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, bool turn, float arc_length, float radian);
    ihmc_msgs::FootstepDataListRosMessage
    createLowerBodyEndStepQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg);
    ihmc_msgs::FootstepDataListRosMessage
    createLowerBodyEndStepStairQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, float height);
    
	
};
 
#endif
