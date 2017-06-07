#ifndef _TRAJECTORY_GENERATION_HPP
#define _TRAJECTORY_GENERATION_HPP

#include "ros/ros.h"
#include "ihmc_msgs/FootstepDataListRosMessage.h"
#include "ihmc_msgs/FootstepDataRosMessage.h"
#include "ihmc_msgs/HeadTrajectoryRosMessage.h"
#include "ihmc_msgs/NeckTrajectoryRosMessage.h"
#include "ihmc_msgs/WholeBodyTrajectoryRosMessage.h"
#include "ihmc_msgs/ArmTrajectoryRosMessage.h"
#include "ihmc_msgs/HandTrajectoryRosMessage.h"
#include "ihmc_msgs/ChestTrajectoryRosMessage.h"
#include "ihmc_msgs/PelvisTrajectoryRosMessage.h"
#include "ihmc_msgs/FootTrajectoryRosMessage.h"
#include "ihmc_msgs/OneDoFJointTrajectoryRosMessage.h"
#include "ihmc_msgs/TrajectoryPoint1DRosMessage.h"
#include "ihmc_msgs/Point2dRosMessage.h"
#include "ihmc_msgs/SO3TrajectoryPointRosMessage.h"
#include "ihmc_msgs/SE3TrajectoryPointRosMessage.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"
#include "math.h"

class Trajectory_Generation
{
  public :
    const bool LEFT = 0;
    const bool RIGHT = 1;
    const std::string WORLD = "world";
    const std::string RIGHT_FOOT_ANKLE = "rightFoot";
    const std::string LEFT_FOOT_ANKLE = "leftFoot";
    const std::string RIGHT_FOOT_SOLE = "rightCOP_Frame";
    const std::string LEFT_FOOT_SOLE = "leftCOP_Frame";
    const std::string PELVIS = "pelvis";
    const float foot_y_seperation = 0.28;
    const float FOOT_LENGTH = 0.27;
    const float FOOT_WIDTH = 0.16;
  
    Trajectory_Generation();
	ihmc_msgs::SO3TrajectoryPointRosMessage
	createSO3TrajectoryOrient(float time, std::vector<float> orient, std::vector<float> ang_vel);
	ihmc_msgs::ArmTrajectoryRosMessage
	appendTrajectoryPoint(float time, std::vector<float> trajectory, std::vector<float> ang_vel, ihmc_msgs::ArmTrajectoryRosMessage arm_msg);
	std_msgs::Float64MultiArray
	createFingerTrajectory(std::vector<float> joint_positions);
    ihmc_msgs::NeckTrajectoryRosMessage
    appendTrajectoryPoint(float time, std::vector<float> trajectory, std::vector<float> ang_vel, ihmc_msgs::NeckTrajectoryRosMessage neck_msg);
	ihmc_msgs::SE3TrajectoryPointRosMessage
	createPelvisHeightTrajectoryPoint(float time, float height);
	ihmc_msgs::SE3TrajectoryPointRosMessage
	createPelvisTrajectoryPointOffset(float time, std::vector<float> pos, std::vector<float> orient, std::vector<float> line_vel, std::vector<float> ang_vel);
	ihmc_msgs::SE3TrajectoryPointRosMessage
	createSE3(float time, std::vector<float> pos, std::vector<float> orient, std::vector<float> line_vel, std::vector<float> ang_vel);
	ihmc_msgs::FootstepDataRosMessage
	createStep(bool step_side, float step_distance, std::vector<float> pos_next_step);
	ihmc_msgs::FootstepDataRosMessage
	createStepOffset(bool step_side, float step_distance, std::vector<float> pos_next_step);
    ihmc_msgs::FootstepDataRosMessage
    createStepArc(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading);
    ihmc_msgs::FootstepDataRosMessage
    createStepOffsetArc(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading);
    ihmc_msgs::FootstepDataRosMessage
    createStepLinear(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading);
    ihmc_msgs::FootstepDataRosMessage
    createStepLinearStairs(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading);
    ihmc_msgs::FootstepDataRosMessage
    createStepOffsetLinear(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading);
    ihmc_msgs::FootstepDataRosMessage
    createStepOffsetReset(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading);
};
 
#endif
