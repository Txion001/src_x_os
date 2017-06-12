#include "satellite_dish_driver/satellite_dish_driver.hpp"
#include "math.h"
#include "x_publisher/x_publisher.hpp"
#include "ihmc_msgs/ArmTrajectoryRosMessage.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"

Satellite_Dish_Driver::Satellite_Dish_Driver()
{
  stepListPublisher = dishNodeHandle.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list", 1);
  trajectoryListPublisher = dishNodeHandle.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/whole_body_trajectory", 1);
  trajectoryNeckPublisher = dishNodeHandle.advertise<ihmc_msgs::NeckTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/neck_trajectory", 1);
  trajectoryRightHandPublisher = dishNodeHandle.advertise<std_msgs::Float64MultiArray>("/right_hand_position_controller/command", 1);
  trajectoryLeftHandPublisher = dishNodeHandle.advertise<std_msgs::Float64MultiArray>("/left_hand_position_controller/command", 1);
  dishNodeHandle.setCallbackQueue(&dish_angle_sub_callback);
  dish_angle_sub = dishNodeHandle.subscribe("/task1/checkpoint2/satellite", 1, &Satellite_Dish_Driver::getDishAngle, this);
}

void
Satellite_Dish_Driver::getDishAngle(const srcsim::Satellite& msg)
{
  target_pitch = msg.target_pitch;
  target_yaw = msg.target_yaw;
  current_pitch = msg.current_pitch;
  current_yaw = msg.current_yaw;
  pitch_completed = msg.pitch_completed;
  yaw_completed = msg.yaw_completed;
}

std::vector<float>
Satellite_Dish_Driver::grabTableTrajectory(bool robot_side_arm, int step_num)
{
  std::vector<float> trajectory_out(7, 0);
  if(step_num == 0)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.4;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 0.4;
      trajectory_out[3] = -0.5;
      trajectory_out[4] = -0.4;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -0.4;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 0.4;
      trajectory_out[3] = 0.5;
      trajectory_out[4] = -0.4;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 1)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.8;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 0.4;
      trajectory_out[3] = -0.4;
      trajectory_out[4] = -0.4;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.4;
    }
    else
    {
      trajectory_out[0] = -0.8;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 0.4;
      trajectory_out[3] = 0.4;
      trajectory_out[4] = -0.4;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.4;
    }
  }
  if(step_num == 2)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.6;
      trajectory_out[1] = -1.0;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = -0.35;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.4;
    }
    else
    {
      trajectory_out[0] = -0.6;
      trajectory_out[1] = 1.0;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 0.35;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.4;
    }
  }
  else if(step_num == 3)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.8;
      trajectory_out[1] = -1.4;
      trajectory_out[2] = 0.4;
      trajectory_out[3] = -0.35;
      trajectory_out[4] = 1.17;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.4;
    }
    else
    {
      trajectory_out[0] = -0.8;
      trajectory_out[1] = 1.4;
      trajectory_out[2] = 0.4;
      trajectory_out[3] = 0.35;
      trajectory_out[4] = 1.17;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.4;
    }
  }
  else if(step_num == 4)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = 0.2;
      trajectory_out[1] = -1.20;
      trajectory_out[2] = -0.3;
      trajectory_out[3] = -1.3;
      trajectory_out[4] = 0.8;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = 0.2;
      trajectory_out[1] = 1.20;
      trajectory_out[2] = -0.3;
      trajectory_out[3] = 1.3;
      trajectory_out[4] = 0.8;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 5)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.6;
      trajectory_out[1] = -1.0;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = -0.8;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
    else
    {
      trajectory_out[0] = -0.6;
      trajectory_out[1] = 1.0;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 0.8;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.2;
    }
  }
  else if(step_num == 6)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.0;
      trajectory_out[1] = -1.4;
      trajectory_out[2] = 0.4;
      trajectory_out[3] = -0.35;
      trajectory_out[4] = 1.17;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
    else
    {
      trajectory_out[0] = -1.0;
      trajectory_out[1] = 1.4;
      trajectory_out[2] = 0.4;
      trajectory_out[3] = 0.35;
      trajectory_out[4] = 1.17;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.2;
    }
  }
  if(step_num == 7)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.3;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 0.8;
      trajectory_out[3] = -0.8;
      trajectory_out[4] = -0.4;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -0.3;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 0.8;
      trajectory_out[3] = 0.8;
      trajectory_out[4] = -0.4;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  return trajectory_out;
}

std::vector<float>
Satellite_Dish_Driver::moveHandlesTrajectory(bool robot_side_arm, int step_num)
{
  std::vector<float> trajectory_out(7, 0);
  if(step_num == 0)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.0;
      trajectory_out[1] = -1.34;
      trajectory_out[2] = 0.3;
      trajectory_out[3] = -0.8;
      trajectory_out[4] = 1.5;
      trajectory_out[5] = -0.40;
      trajectory_out[6] = 0.28;
    }
    else
    {
      trajectory_out[0] = -1.0;
      trajectory_out[1] = 1.34;
      trajectory_out[2] = 0.3;
      trajectory_out[3] = 0.8;
      trajectory_out[4] = 1.5;
      trajectory_out[5] = 0.40;
      trajectory_out[6] = 0.28;
    }
  }
  if(step_num == 1)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.25;
      trajectory_out[1] = -1.3;
      trajectory_out[2] = 0.3;
      trajectory_out[3] = -1.2;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = -0.60;
      trajectory_out[6] = -0.28;
    }
    else
    {
      trajectory_out[0] = -0.25;
      trajectory_out[1] = 1.28;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 1.2;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.60;
      trajectory_out[6] = 0.28;
    }
  }
  if(step_num == 2)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.5;
      trajectory_out[1] = -1.0;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = -1.1;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = -0.0;
      trajectory_out[6] = -0.2;
    }
    else
    {
      trajectory_out[0] = -0.5;
      trajectory_out[1] = 1.0;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 1.1;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.2;
    }
  }
  if(step_num == 3)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.7;
      trajectory_out[1] = -1.0;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = -1.0;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = -0.0;
      trajectory_out[6] = -0.2;
    }
    else
    {
      trajectory_out[0] = -0.7;
      trajectory_out[1] = 1.0;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 1.0;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.2;
    }
  }
  if(step_num == 4)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.2;
      trajectory_out[1] = -1.3;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = -1.3;
      trajectory_out[4] = 1.3;
      trajectory_out[5] = 0.6;
      trajectory_out[6] = 0.2;
    }
    else
    {
      trajectory_out[0] = -0.2;
      trajectory_out[1] = 1.3;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 1.3;
      trajectory_out[4] = 1.3;
      trajectory_out[5] = -0.6;
      trajectory_out[6] = -0.2;
    }
  }
  if(step_num == 5)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.8;
      trajectory_out[1] = -1.34;
      trajectory_out[2] = 0.6;
      trajectory_out[3] = -0.9;
      trajectory_out[4] = 1.5;
      trajectory_out[5] = 0.60;
      trajectory_out[6] = -0.28;
    }
    else
    {
      trajectory_out[0] = -0.8;
      trajectory_out[1] = 1.34;
      trajectory_out[2] = 0.6;
      trajectory_out[3] = 0.9;
      trajectory_out[4] = 1.5;
      trajectory_out[5] = -0.60;
      trajectory_out[6] = 0.28;
    }
  }
  if(step_num == 6)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.5;
      trajectory_out[1] = -1.0;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = -1.1;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = -0.0;
      trajectory_out[6] = -0.2;
    }
    else
    {
      trajectory_out[0] = -0.5;
      trajectory_out[1] = 1.0;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 1.1;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.2;
    }
  }
  return trajectory_out;
}
std::vector<float>
Satellite_Dish_Driver::getArmZeroAngVel()
{
  std::vector<float> arm_zero_ang_vel(7, 0);
  return arm_zero_ang_vel;
}

void
Satellite_Dish_Driver::grabTableBottom()
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped pelvis = tfBuffer.lookupTransform(WORLD, PELVIS, ros::Time(0), ros::Duration(0.2));
  
  tf2::Quaternion heading(pelvis.transform.rotation.x, pelvis.transform.rotation.y, pelvis.transform.rotation.z, pelvis.transform.rotation.w);
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Matrix3x3 rotation(heading);
  rotation.getRPY(roll, pitch, yaw);
  rotation.setRPY(0.0, 0.0, yaw);
  
  // Set Valkyrie into starting configuration
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  
  // Set unique ids to feature used messages
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 1;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 1;
  pelvis_msg.unique_id = 1;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  // Set sides per message group
  left_arm_msg.robot_side = LEFT;
  right_arm_msg.robot_side = RIGHT;
  left_hand_msg.robot_side = LEFT;
  right_hand_msg.robot_side = RIGHT;
  left_foot_msg.robot_side = LEFT;
  right_foot_msg.robot_side = RIGHT;
  
  std::vector<float> left_arm_swing = grabTableTrajectory(LEFT, 7);
  std::vector<float> right_arm_swing = grabTableTrajectory(RIGHT, 7);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  
  for(int grab_step_num = 0; grab_step_num < 2; grab_step_num++)
  {
    left_arm_swing = grabTableTrajectory(LEFT, grab_step_num);
    right_arm_swing = grabTableTrajectory(RIGHT, grab_step_num);
    left_arm_msg = trajectory_generation.appendTrajectoryPoint(grab_step_num+3, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
    right_arm_msg = trajectory_generation.appendTrajectoryPoint(grab_step_num+3, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  }
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.3;
  new_chest_orient[2] = yaw;
  float chest_time = 1.0;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(0.0, new_chest_orient, new_chest_ang_vel));
  
  pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(0.5, 0.95));
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.0;
  finger_pos[1] = -0.8;
  finger_pos[2] = -0.0;
  finger_pos[3] = -0.0;
  finger_pos[4] = -0.0;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  finger_pos[0] = 0.0;
  finger_pos[1] = 0.8;
  finger_pos[2] = 0.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  
  trajectoryListPublisher.publish(trajectory_list_msg);
  trajectoryRightHandPublisher.publish(right_fingers);
  trajectoryLeftHandPublisher.publish(left_fingers);
}

void
Satellite_Dish_Driver::grabTableSide()
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped pelvis = tfBuffer.lookupTransform(WORLD, PELVIS, ros::Time(0), ros::Duration(0.2));
  
  tf2::Quaternion heading(pelvis.transform.rotation.x, pelvis.transform.rotation.y, pelvis.transform.rotation.z, pelvis.transform.rotation.w);
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Matrix3x3 rotation(heading);
  rotation.getRPY(roll, pitch, yaw);
  rotation.setRPY(0.0, 0.0, yaw);
  
  // Set Valkyrie into starting configuration
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  
  // Set unique ids to feature used messages
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 1;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 1;
  pelvis_msg.unique_id = 1;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  // Set sides per message group
  left_arm_msg.robot_side = LEFT;
  right_arm_msg.robot_side = RIGHT;
  left_hand_msg.robot_side = LEFT;
  right_hand_msg.robot_side = RIGHT;
  left_foot_msg.robot_side = LEFT;
  right_foot_msg.robot_side = RIGHT;
  
  for(int grab_step_num = 2; grab_step_num < 4; grab_step_num++)
  {
    std::vector<float> left_arm_swing = grabTableTrajectory(LEFT, grab_step_num);
    std::vector<float> right_arm_swing = grabTableTrajectory(RIGHT, grab_step_num);
    left_arm_msg = trajectory_generation.appendTrajectoryPoint(grab_step_num-1, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
    right_arm_msg = trajectory_generation.appendTrajectoryPoint(grab_step_num-1, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  }
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.3;
  new_chest_orient[2] = yaw;
  float chest_time = 1.0;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(0.0, new_chest_orient, new_chest_ang_vel));
  
  pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(0.5, 0.95));
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> neck_default(3, 0);
  std::vector<float> neck_default_ang_vel(3, 0);
  float neck_pitch = 0.6;
  neck_default[0] = 0.431 - neck_pitch*0.431/(M_PI/2);
  neck_default[1] = 0.0;
  neck_default[2] = -0.431 - neck_pitch*0.431/(M_PI/2);
  neck_msg= trajectory_generation.appendTrajectoryPoint(0.0, neck_default, neck_default_ang_vel, neck_msg);
  
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.5;
  finger_pos[1] = -0.1;
  finger_pos[2] = -0.6;
  finger_pos[3] = -0.7;
  finger_pos[4] = -0.8;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  finger_pos[0] = 0.5;
  finger_pos[1] = 0.1;
  finger_pos[2] = 0.6;
  finger_pos[3] = 0.7;
  finger_pos[4] = 0.8;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  
  trajectoryListPublisher.publish(trajectory_list_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  ros::Duration(2.0).sleep();
  trajectoryRightHandPublisher.publish(right_fingers);
  trajectoryLeftHandPublisher.publish(left_fingers);
}

void
Satellite_Dish_Driver::checkTableSide()
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped pelvis = tfBuffer.lookupTransform(WORLD, PELVIS, ros::Time(0), ros::Duration(0.2));
  
  tf2::Quaternion heading(pelvis.transform.rotation.x, pelvis.transform.rotation.y, pelvis.transform.rotation.z, pelvis.transform.rotation.w);
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Matrix3x3 rotation(heading);
  rotation.getRPY(roll, pitch, yaw);
  rotation.setRPY(0.0, 0.0, yaw);
  
  // Set Valkyrie into starting configuration
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  
  // Set unique ids to feature used messages
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 1;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 1;
  pelvis_msg.unique_id = 1;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  // Set sides per message group
  left_arm_msg.robot_side = LEFT;
  right_arm_msg.robot_side = RIGHT;
  left_hand_msg.robot_side = LEFT;
  right_hand_msg.robot_side = RIGHT;
  left_foot_msg.robot_side = LEFT;
  right_foot_msg.robot_side = RIGHT;
  
  for(int grab_step_num = 5; grab_step_num < 7; grab_step_num++)
  {
    std::vector<float> left_arm_swing = grabTableTrajectory(LEFT, grab_step_num);
    std::vector<float> right_arm_swing = grabTableTrajectory(RIGHT, grab_step_num);
    left_arm_msg = trajectory_generation.appendTrajectoryPoint(grab_step_num-4, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
    right_arm_msg = trajectory_generation.appendTrajectoryPoint(grab_step_num-4, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  }
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.3;
  new_chest_orient[2] = yaw;
  float chest_time = 1.0;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(0.0, new_chest_orient, new_chest_ang_vel));
  
  pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(0.5, 0.95));
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> neck_default(3, 0);
  std::vector<float> neck_default_ang_vel(3, 0);
  float neck_pitch = 0.6;
  neck_default[0] = 0.431 - neck_pitch*0.431/(M_PI/2);
  neck_default[1] = 0.0;
  neck_default[2] = -0.431 - neck_pitch*0.431/(M_PI/2);
  neck_msg= trajectory_generation.appendTrajectoryPoint(0.0, neck_default, neck_default_ang_vel, neck_msg);
  
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.0;
  finger_pos[1] = -0.0;
  finger_pos[2] = -0.0;
  finger_pos[3] = -0.0;
  finger_pos[4] = -0.0;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  finger_pos[0] = 0.0;
  finger_pos[1] = 0.0;
  finger_pos[2] = 0.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  
  trajectoryListPublisher.publish(trajectory_list_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  trajectoryRightHandPublisher.publish(right_fingers);
  trajectoryLeftHandPublisher.publish(left_fingers);
}

void
Satellite_Dish_Driver::letGoTable()
{
  // Set Valkyrie into starting configuration
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  
  // Set unique ids to feature used messages
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 1;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 0;
  pelvis_msg.unique_id = 1;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  // Set sides per message group
  left_arm_msg.robot_side = LEFT;
  right_arm_msg.robot_side = RIGHT;
  left_hand_msg.robot_side = LEFT;
  right_hand_msg.robot_side = RIGHT;
  left_foot_msg.robot_side = LEFT;
  right_foot_msg.robot_side = RIGHT;
  
  for(int grab_step_num = 4; grab_step_num < 5; grab_step_num++)
  {
    std::vector<float> left_arm_swing = grabTableTrajectory(LEFT, grab_step_num);
    std::vector<float> right_arm_swing = grabTableTrajectory(RIGHT, grab_step_num);
    left_arm_msg = trajectory_generation.appendTrajectoryPoint(2.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
    right_arm_msg = trajectory_generation.appendTrajectoryPoint(2.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  }
  
  pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(1.0, 0.95));
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.0;
  finger_pos[1] = -0.6;
  finger_pos[2] = -0.0;
  finger_pos[3] = -0.0;
  finger_pos[4] = -0.0;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  finger_pos[0] = 0.0;
  finger_pos[1] = 0.6;
  finger_pos[2] = 0.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  
  trajectoryListPublisher.publish(trajectory_list_msg);
  trajectoryRightHandPublisher.publish(right_fingers);
  trajectoryLeftHandPublisher.publish(left_fingers);
  ros::Duration(1.0).sleep();
}

void
Satellite_Dish_Driver::centerBetweenArms()
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  X_Publisher x_publisher;
  
  geometry_msgs::TransformStamped torso = tfBuffer.lookupTransform(WORLD, TORSO, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped left_wrist = tfBuffer.lookupTransform(PELVIS, LEFT_WRIST, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_wrist = tfBuffer.lookupTransform(PELVIS, RIGHT_WRIST, ros::Time(0), ros::Duration(0.2));
  
  geometry_msgs::Vector3 center_point = left_wrist.transform.translation;
  center_point.x += right_wrist.transform.translation.x;
  center_point.y += right_wrist.transform.translation.y;
  center_point.z = 0.0;
  ROS_INFO("YY:\t%f\t%f", left_wrist.transform.translation.y, right_wrist.transform.translation.y);
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  ROS_INFO("XY:\t%f\t%f", center_point.x, center_point.y);
  letGoTable();
  ros::Duration(1.5).sleep();
  
  //// Set Valkyrie into starting configuration
  //ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  //ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  //ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  //ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  //ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  //ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  //ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  //ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  //ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  //
  //// Set unique ids to feature used messages
  //trajectory_list_msg.unique_id = 1;
  //left_arm_msg.unique_id = 0;
  //right_arm_msg.unique_id = 0;
  //left_hand_msg.unique_id = 0;
  //right_hand_msg.unique_id = 0;
  //chest_msg.unique_id = 0;
  //pelvis_msg.unique_id = 1;
  //left_foot_msg.unique_id = 0;
  //right_foot_msg.unique_id = 0;
  //
  //// Set sides per message group
  //left_arm_msg.robot_side = LEFT;
  //right_arm_msg.robot_side = RIGHT;
  //left_hand_msg.robot_side = LEFT;
  //right_hand_msg.robot_side = RIGHT;
  //left_foot_msg.robot_side = LEFT;
  //right_foot_msg.robot_side = RIGHT;
  //
  //std::vector<float> pos(3, 0);
  //std::vector<float> orient(3, 0);
  //std::vector<float> line_vel(3, 0);
  //std::vector<float> ang_vel(3, 0);
  //pos[1] = center_point.y;
  //pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  //pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisTrajectoryPointOffset(0.5, pos, orient, line_vel, ang_vel));
  //
  ////Publish All Trajectories
  //trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  //trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  //trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  //trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  //trajectory_list_msg.chest_trajectory_message = chest_msg;
  //trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  //trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  //trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  //
  //trajectoryListPublisher.publish(trajectory_list_msg);
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(0.01);
  x_vector end_point_location;
  end_point_location.x = 0;
  end_point_location.y = center_point.y;
  end_point_location.z = 0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOffsetQueue(step_list_msg, end_point_location, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  x_publisher.publish(step_list_msg);
}

void
Satellite_Dish_Driver::alignArms()
{
  X_Publisher x_publisher;
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  geometry_msgs::TransformStamped torso = tfBuffer.lookupTransform(WORLD, TORSO, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped left_wrist = tfBuffer.lookupTransform(WORLD, LEFT_WRIST, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_wrist = tfBuffer.lookupTransform(WORLD, RIGHT_WRIST, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  
  float hand_seperation = sqrt(pow(fabs(left_wrist.transform.translation.x-right_wrist.transform.translation.x), 2.0) + pow(fabs(left_wrist.transform.translation.y-right_wrist.transform.translation.y), 2.0));
  float hand_heading = asin((left_wrist.transform.translation.y-right_wrist.transform.translation.y)/hand_seperation);
  if(left_wrist.transform.translation.x-right_wrist.transform.translation.x <= 0)
  {
    if(left_wrist.transform.translation.y-right_wrist.transform.translation.y <= 0)
    {
      hand_heading = -M_PI-hand_heading;
    }
    else
    {
      hand_heading = M_PI-hand_heading;
    }
  }
  hand_heading -= M_PI/2;
  tf2Scalar left_roll;
  tf2Scalar left_pitch;
  tf2Scalar left_yaw;
  tf2Scalar right_roll;
  tf2Scalar right_pitch;
  tf2Scalar right_yaw;
  tf2::Quaternion left_orientation(left_foot_ankle.transform.rotation.x, left_foot_ankle.transform.rotation.y, left_foot_ankle.transform.rotation.z, left_foot_ankle.transform.rotation.w);
  tf2::Quaternion right_orientation(right_foot_ankle.transform.rotation.x, right_foot_ankle.transform.rotation.y, right_foot_ankle.transform.rotation.z, right_foot_ankle.transform.rotation.w);
  tf2::Matrix3x3 left_rotation(left_orientation);
  left_rotation.getRPY(left_roll, left_pitch, left_yaw);
  tf2::Matrix3x3 right_rotation(right_orientation);
  right_rotation.getRPY(right_roll, right_pitch, right_yaw);
  tf2Scalar roll = (left_roll+right_roll)/2;
  tf2Scalar pitch = (left_pitch+right_pitch)/2;
  tf2Scalar yaw = (left_yaw+right_yaw)/2;
  ROS_INFO("headings:\t%f\t%f", hand_heading, yaw);
  int num_angle = round(hand_heading / M_PI_4);
  hand_heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", hand_heading);
  ros::Duration(0.01).sleep();
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(hand_heading - yaw);
  
  left_wrist = tfBuffer.lookupTransform(LEFT_FOOT_ANKLE, LEFT_WRIST, ros::Time(0), ros::Duration(0.2));
  right_wrist = tfBuffer.lookupTransform(LEFT_FOOT_ANKLE, RIGHT_WRIST, ros::Time(0), ros::Duration(0.2));
  
  geometry_msgs::Vector3 center_point = left_wrist.transform.translation;
  center_point.x += right_wrist.transform.translation.x;
  center_point.y += right_wrist.transform.translation.y + 0.11;
  center_point.z = 0.0;
  ROS_INFO("YY:\t%f\t%f", left_wrist.transform.translation.y, right_wrist.transform.translation.y);
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  ROS_INFO("XY:\t%f\t%f", center_point.x, center_point.y);
  x_vector destination;
  destination.x = center_point.x - 0.35;
  destination.y = 0;
  destination.z = 0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOffsetQueue(step_list_msg, destination, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  letGoTable();
  x_publisher.publish(step_list_msg);
}

void
Satellite_Dish_Driver::startPitchController()
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // Set Valkyrie into starting configuration
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  
  // Set unique ids to feature used messages
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 1;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 0;
  pelvis_msg.unique_id = 0;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  // Set sides per message group
  left_arm_msg.robot_side = LEFT;
  right_arm_msg.robot_side = RIGHT;
  left_hand_msg.robot_side = LEFT;
  right_hand_msg.robot_side = RIGHT;
  left_foot_msg.robot_side = LEFT;
  right_foot_msg.robot_side = RIGHT;
  
  std::vector<float> left_arm_swing = grabTableTrajectory(LEFT, 3);
  std::vector<float> right_arm_swing = moveHandlesTrajectory(RIGHT, 3);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.5;
  finger_pos[1] = -0.1;
  finger_pos[2] = -0.6;
  finger_pos[3] = -0.7;
  finger_pos[4] = -0.8;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  finger_pos[0] = 0.3;
  finger_pos[1] = 0.0;
  finger_pos[2] = 0.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  
  trajectoryListPublisher.publish(trajectory_list_msg);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(1.0).sleep();
  trajectoryLeftHandPublisher.publish(left_fingers);
}

void
Satellite_Dish_Driver::increasePitch()
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // Set Valkyrie into starting configuration
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  
  // Set unique ids to feature used messages
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 1;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 0;
  pelvis_msg.unique_id = 0;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  // Set sides per message group
  left_arm_msg.robot_side = LEFT;
  right_arm_msg.robot_side = RIGHT;
  left_hand_msg.robot_side = LEFT;
  right_hand_msg.robot_side = RIGHT;
  left_foot_msg.robot_side = LEFT;
  right_foot_msg.robot_side = RIGHT;
  
  std::vector<float> left_arm_swing = grabTableTrajectory(LEFT, 3);
  std::vector<float> right_arm_swing = moveHandlesTrajectory(RIGHT, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  right_arm_swing = moveHandlesTrajectory(RIGHT, 1);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(2.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  right_arm_swing = moveHandlesTrajectory(RIGHT, 2);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(3.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.5;
  finger_pos[1] = -0.1;
  finger_pos[2] = -0.6;
  finger_pos[3] = -0.7;
  finger_pos[4] = -0.8;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  finger_pos[0] = 0.6;
  finger_pos[1] = 0.4;
  finger_pos[2] = 0.4;
  finger_pos[3] = 0.4;
  finger_pos[4] = 0.4;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  
  trajectoryListPublisher.publish(trajectory_list_msg);
  ros::Duration(0.4).sleep();
  trajectoryRightHandPublisher.publish(right_fingers);
  trajectoryLeftHandPublisher.publish(left_fingers);
  
  ros::Duration(1.0).sleep();
  finger_pos[0] = 0.6;
  finger_pos[1] = 0.4;
  finger_pos[2] = 0.2;
  finger_pos[3] = 0.2;
  finger_pos[4] = 0.2;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
}

void
Satellite_Dish_Driver::decreasePitch()
{
  // Set Valkyrie into starting configuration
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  
  // Set unique ids to feature used messages
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 1;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 0;
  pelvis_msg.unique_id = 0;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  // Set sides per message group
  left_arm_msg.robot_side = LEFT;
  right_arm_msg.robot_side = RIGHT;
  left_hand_msg.robot_side = LEFT;
  right_hand_msg.robot_side = RIGHT;
  left_foot_msg.robot_side = LEFT;
  right_foot_msg.robot_side = RIGHT;
  
  std::vector<float> left_arm_swing = grabTableTrajectory(LEFT, 3);
  std::vector<float> right_arm_swing = moveHandlesTrajectory(RIGHT, 4);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  right_arm_swing = moveHandlesTrajectory(RIGHT, 5);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(2.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  right_arm_swing = moveHandlesTrajectory(RIGHT, 6);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(3.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.5;
  finger_pos[1] = -0.1;
  finger_pos[2] = -0.6;
  finger_pos[3] = -0.7;
  finger_pos[4] = -0.8;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  finger_pos[0] = 0.2;
  finger_pos[1] = 0.1;
  finger_pos[2] = 0.3;
  finger_pos[3] = 0.3;
  finger_pos[4] = 0.3;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  
  trajectoryListPublisher.publish(trajectory_list_msg);
  trajectoryRightHandPublisher.publish(right_fingers);
  trajectoryLeftHandPublisher.publish(left_fingers);
  
  ros::Duration(1.2).sleep();
  finger_pos[0] = 0.2;
  finger_pos[1] = 0.1;
  finger_pos[2] = 0.2;
  finger_pos[3] = 0.2;
  finger_pos[4] = 0.2;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
}

void
Satellite_Dish_Driver::startYawController()
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // Set Valkyrie into starting configuration
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  
  // Set unique ids to feature used messages
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 1;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 0;
  pelvis_msg.unique_id = 0;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  // Set sides per message group
  left_arm_msg.robot_side = LEFT;
  right_arm_msg.robot_side = RIGHT;
  left_hand_msg.robot_side = LEFT;
  right_hand_msg.robot_side = RIGHT;
  left_foot_msg.robot_side = LEFT;
  right_foot_msg.robot_side = RIGHT;
  
  std::vector<float> right_arm_swing = grabTableTrajectory(RIGHT, 3);
  std::vector<float> left_arm_swing = moveHandlesTrajectory(LEFT, 3);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.3;
  finger_pos[1] = -0.0;
  finger_pos[2] = -0.0;
  finger_pos[3] = -0.0;
  finger_pos[4] = -0.0;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  finger_pos[0] = 0.5;
  finger_pos[1] = 0.1;
  finger_pos[2] = 0.6;
  finger_pos[3] = 0.7;
  finger_pos[4] = 0.8;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  
  trajectoryListPublisher.publish(trajectory_list_msg);
  trajectoryLeftHandPublisher.publish(left_fingers);
  ros::Duration(1.0).sleep();
  trajectoryRightHandPublisher.publish(right_fingers);
}

void
Satellite_Dish_Driver::decreaseYaw()
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // Set Valkyrie into starting configuration
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  
  // Set unique ids to feature used messages
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 1;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 0;
  pelvis_msg.unique_id = 0;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  // Set sides per message group
  left_arm_msg.robot_side = LEFT;
  right_arm_msg.robot_side = RIGHT;
  left_hand_msg.robot_side = LEFT;
  right_hand_msg.robot_side = RIGHT;
  left_foot_msg.robot_side = LEFT;
  right_foot_msg.robot_side = RIGHT;
  
  std::vector<float> right_arm_swing = grabTableTrajectory(RIGHT, 3);
  std::vector<float> left_arm_swing = moveHandlesTrajectory(LEFT, 0);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
  left_arm_swing = moveHandlesTrajectory(LEFT, 1);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(2.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
  left_arm_swing = moveHandlesTrajectory(LEFT, 2);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(3.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.5;
  finger_pos[1] = 0.1;
  finger_pos[2] = 0.6;
  finger_pos[3] = 0.7;
  finger_pos[4] = 0.8;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  finger_pos[0] = 0.6;
  finger_pos[1] = -0.4;
  finger_pos[2] = -0.4;
  finger_pos[3] = -0.4;
  finger_pos[4] = -0.4;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  
  trajectoryListPublisher.publish(trajectory_list_msg);
  ros::Duration(0.4).sleep();
  trajectoryRightHandPublisher.publish(right_fingers);
  trajectoryLeftHandPublisher.publish(left_fingers);
  
  ros::Duration(1.0).sleep();
  finger_pos[0] = 0.6;
  finger_pos[1] = -0.4;
  finger_pos[2] = -0.2;
  finger_pos[3] = -0.2;
  finger_pos[4] = -0.2;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
}

void
Satellite_Dish_Driver::increaseYaw()
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // Set Valkyrie into starting configuration
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  
  // Set unique ids to feature used messages
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 1;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 0;
  pelvis_msg.unique_id = 0;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  // Set sides per message group
  left_arm_msg.robot_side = LEFT;
  right_arm_msg.robot_side = RIGHT;
  left_hand_msg.robot_side = LEFT;
  right_hand_msg.robot_side = RIGHT;
  left_foot_msg.robot_side = LEFT;
  right_foot_msg.robot_side = RIGHT;
  
  std::vector<float> right_arm_swing = grabTableTrajectory(RIGHT, 3);
  std::vector<float> left_arm_swing = moveHandlesTrajectory(LEFT, 4);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
  left_arm_swing = moveHandlesTrajectory(LEFT, 5);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(2.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
  left_arm_swing = moveHandlesTrajectory(LEFT, 6);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(3.0, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.5;
  finger_pos[1] = 0.1;
  finger_pos[2] = 0.6;
  finger_pos[3] = 0.7;
  finger_pos[4] = 0.8;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  finger_pos[0] = 0.2;
  finger_pos[1] = -0.1;
  finger_pos[2] = -0.3;
  finger_pos[3] = -0.3;
  finger_pos[4] = -0.3;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  
  trajectoryListPublisher.publish(trajectory_list_msg);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(0.2).sleep();
  trajectoryLeftHandPublisher.publish(left_fingers);
  
  ros::Duration(1.0).sleep();
  finger_pos[0] = 0.2;
  finger_pos[1] = -0.1;
  finger_pos[2] = -0.2;
  finger_pos[3] = -0.2;
  finger_pos[4] = -0.2;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
}

void
Satellite_Dish_Driver::alignDish()
{
  ros::NodeHandle alignDishNodeHandle;
  // Starts of with aligning the pitch
  startPitchController();
  ros::Duration(2.0).sleep();
  
  //Runs a loop to rotate handles untill desired point has been reached
  while(pitch_completed == false && alignDishNodeHandle.ok())
  {
    // Obtain all values from dish
    dish_angle_sub_callback.callAvailable(ros::WallDuration());
    
    // Rotates handles accordingly to values
    if(target_pitch > current_pitch + 0.05)
    {
      increasePitch();
    }
    else if(target_pitch < current_pitch - 0.05)
    {
      decreasePitch();
    }
    ros::Duration(2.0).sleep();
  }
  
  // Continues with starting yaw
  startYawController();
  ros::Duration(2.0).sleep();
  
  //Runs a loop to rotate handles untill desired point has been reached
  while(yaw_completed == false && alignDishNodeHandle.ok())
  {
    // Obtain all values from dish
    dish_angle_sub_callback.callAvailable(ros::WallDuration());
    
    // Rotates handles accordingly to values
    if(target_yaw > current_yaw + 0.05)
    {
      increaseYaw();
    }
    else if(target_yaw < current_yaw - 0.05)
    {
      decreaseYaw();
    }
    ros::Duration(2.0).sleep();
  }
}

void
Satellite_Dish_Driver::resetStance(std::vector<float> next_point)
{
  X_Publisher x_publisher;
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  tf2Scalar left_roll;
  tf2Scalar left_pitch;
  tf2Scalar left_yaw;
  tf2Scalar right_roll;
  tf2Scalar right_pitch;
  tf2Scalar right_yaw;
  tf2::Quaternion left_orientation(left_foot_ankle.transform.rotation.x, left_foot_ankle.transform.rotation.y, left_foot_ankle.transform.rotation.z, left_foot_ankle.transform.rotation.w);
  tf2::Quaternion right_orientation(right_foot_ankle.transform.rotation.x, right_foot_ankle.transform.rotation.y, right_foot_ankle.transform.rotation.z, right_foot_ankle.transform.rotation.w);
  tf2::Matrix3x3 left_rotation(left_orientation);
  left_rotation.getRPY(left_roll, left_pitch, left_yaw);
  tf2::Matrix3x3 right_rotation(right_orientation);
  right_rotation.getRPY(right_roll, right_pitch, right_yaw);
  tf2Scalar roll = (left_roll+right_roll)/2;
  tf2Scalar pitch = (left_pitch+right_pitch)/2;
  tf2Scalar yaw = (left_yaw+right_yaw)/2;
  
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
  float distance = sqrt(pow(fabs(next_point[0]-center_point.x), 2.0) + pow(fabs(next_point[1]-center_point.y), 2.0));
  float heading = asin((next_point[1]-center_point.y)/distance);
  if(next_point[0]-center_point.x <= 0)
  {
    if(next_point[1]-center_point.y <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  
  trajectoryListPublisher.publish(gait_generation.createStanceReset().trajectory_list_msg);
  trajectoryNeckPublisher.publish(gait_generation.createStanceReset().trajectory_neck_msg);
  trajectoryLeftHandPublisher.publish(gait_generation.createStanceReset().trajectory_finger_left_msg);
  trajectoryRightHandPublisher.publish(gait_generation.createStanceReset().trajectory_finger_right_msg);
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  x_vector destination;
  destination.x = -0.5;
  destination.y = 0;
  destination.z = 0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOverride(destination, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  if(heading-yaw > 3*M_PI/4 || heading-yaw < -3*M_PI/4)
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, (heading-yaw)/4);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, (heading-yaw)/4);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, (heading-yaw)/4);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, (heading-yaw)/4);
  }
  else if(heading-yaw > M_PI/2 || heading-yaw < -M_PI/2)
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, (heading-yaw)/3);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, (heading-yaw)/3);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, (heading-yaw)/3);
  }
  else if(heading-yaw > M_PI/4 || heading-yaw < -M_PI/4)
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, (heading-yaw)/2);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, (heading-yaw)/2);
  }
  else
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading-yaw);
  }
  x_publisher.publish(step_list_msg);
}
