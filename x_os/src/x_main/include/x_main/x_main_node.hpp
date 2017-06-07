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
#ifndef _X_MAIN_HPP
#define _X_MAIN_HPP

#include "ros/ros.h"
#include "trajectory_generation/trajectory_generation.hpp"
#include "gait_generation/gait_generation.hpp"
#include "src_finals_controller/src_finals_controller.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"
#include "x_msgs/X_Status.h"
#include "x_msgs/X_Command.h"
/*
 * State Machine Decleration:
 * STATES:
 * PASSIVE
 * -SETUP
 * -READY
 * -UNAVAILABLE
 * -FAILURE
 * ACTIVE
 * -SETUP
 * -STANDBY
 * -EXECUTE
 * -SUCCESS
 * -RESET
 */
const std::vector<std::string> PASSIVE_STATE
{
  "SETUP",
  "READY",
  "PROCESSING",
  "FAILURE",
};
const std::vector<std::string> ACTIVE_STATE
{
  "SETUP",
  "STANDBY",
  "EXECUTE",
  "SUCCESS",
  "RESET"
};

std::string passive_state;
std::string active_state;

const bool LEFT = 0;
const bool RIGHT = 1;
const std::string WORLD = "world";
const std::string RIGHT_FOOT_ANKLE = "rightFoot";
const std::string LEFT_FOOT_ANKLE = "leftFoot";
const std::string RIGHT_FOOT_SOLE = "rightCOP_Frame";
const std::string LEFT_FOOT_SOLE = "leftCOP_Frame";
const std::string PELVIS = "pelvis";

int current_command;

ros::Publisher stepListPublisher;
ros::Publisher trajectoryListPublisher;
ros::Publisher trajectoryNeckPublisher;
ros::Publisher trajectoryRightHandPublisher;
ros::Publisher trajectoryLeftHandPublisher;

ros::Publisher statusPublisher;
ros::ServiceServer commandService;
ros::ServiceClient tasksClient;

Trajectory_Generation trajectory_generation;
Gait_Generation gait_generation;

bool
activeReset();
void
activeAutoRun();
bool
recieveCommand(x_msgs::X_Command::Request  &req, x_msgs::X_Command::Response &res);

#endif
