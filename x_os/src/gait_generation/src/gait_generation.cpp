#include "gait_generation/gait_generation.hpp"

Gait_Generation::Gait_Generation()
{
}

full_msg
Gait_Generation::createWakeUp()
{
  full_msg msg_out;
  return msg_out;
}

full_msg
Gait_Generation::createStanceReset()
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  //Sends walking commands to valkyrie
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  
  // Step list message configuration
  step_list_msg.execution_mode = ihmc_msgs::FootstepDataListRosMessage::OVERRIDE;
  step_list_msg.default_swing_time = 1.5;
  step_list_msg.default_transfer_time = 1.5;
  step_list_msg.final_transfer_time = 1.5;
  step_list_msg.unique_id = 1;
  
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped pelvis = tfBuffer.lookupTransform(WORLD, PELVIS, ros::Time(0), ros::Duration(0.2));
  
  
  tf2::Quaternion heading(pelvis.transform.rotation.x, pelvis.transform.rotation.y, pelvis.transform.rotation.z, pelvis.transform.rotation.w);
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Matrix3x3 rotation(heading);
  rotation.getRPY(roll, pitch, yaw);
  rotation.setRPY(0.0, 0.0, yaw);
  
  std::vector<float> new_step_pos_left (3, 0);
  new_step_pos_left[0] = 0.0;
  new_step_pos_left[1] = DEFAULT_STEP_SEPERATION/2;
  new_step_pos_left[2] = 0.0;
  std::vector<float> new_step_pos_right (3, 0);
  new_step_pos_right[0] = 0.0;
  new_step_pos_right[1] = -DEFAULT_STEP_SEPERATION/2;
  new_step_pos_right[2] = 0.0;
  
  x_vector new_step_pos;
  new_step_pos.x = new_step_pos_left[0]*rotation[0][0] + new_step_pos_left[1]*rotation[0][1] + new_step_pos_left[2]*rotation[0][2];
  new_step_pos.y = new_step_pos_left[0]*rotation[1][0] + new_step_pos_left[1]*rotation[1][1] + new_step_pos_left[2]*rotation[1][2];
  new_step_pos.z = new_step_pos_left[0]*rotation[2][0] + new_step_pos_left[1]*rotation[2][1] + new_step_pos_left[2]*rotation[2][2];
  new_step_pos_left[0] = new_step_pos.x;
  new_step_pos_left[1] = new_step_pos.y;
  new_step_pos_left[2] = new_step_pos.z;
  new_step_pos.x = new_step_pos_right[0]*rotation[0][0] + new_step_pos_right[1]*rotation[0][1] + new_step_pos_right[2]*rotation[0][2];
  new_step_pos.y = new_step_pos_right[0]*rotation[1][0] + new_step_pos_right[1]*rotation[1][1] + new_step_pos_right[2]*rotation[1][2];
  new_step_pos.z = new_step_pos_right[0]*rotation[2][0] + new_step_pos_right[1]*rotation[2][1] + new_step_pos_right[2]*rotation[2][2];
  new_step_pos_right[0] = new_step_pos.x;
  new_step_pos_right[1] = new_step_pos.y;
  new_step_pos_right[2] = new_step_pos.z;
  
  if(right_foot_ankle.transform.translation.z > left_foot_ankle.transform.translation.z)
  {
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetReset(RIGHT, 0.0, new_step_pos_right, 0.0));
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetReset(LEFT, 0.0, new_step_pos_left, 0.0));
  }
  else
  {
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetReset(LEFT, 0.0, new_step_pos_left, 0.0));
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetReset(RIGHT, 0.0, new_step_pos_right, 0.0));
  }
  
  //Sets Valkyrie into starting configuration
  ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg;
  ihmc_msgs::ArmTrajectoryRosMessage left_arm_msg;
  ihmc_msgs::ArmTrajectoryRosMessage right_arm_msg;
  ihmc_msgs::HandTrajectoryRosMessage left_hand_msg;
  ihmc_msgs::HandTrajectoryRosMessage right_hand_msg;
  ihmc_msgs::ChestTrajectoryRosMessage chest_msg;
  ihmc_msgs::PelvisTrajectoryRosMessage pelvis_msg;
  ihmc_msgs::FootTrajectoryRosMessage left_foot_msg;
  ihmc_msgs::FootTrajectoryRosMessage right_foot_msg;
  
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 1;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 1;
  pelvis_msg.unique_id = 1;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  left_arm_msg.robot_side = LEFT;
  right_arm_msg.robot_side = RIGHT;
  left_hand_msg.robot_side = LEFT;
  right_hand_msg.robot_side = RIGHT;
  left_foot_msg.robot_side = LEFT;
  right_foot_msg.robot_side = RIGHT;
  
  std::vector<float> left_arm_set(7, 0);
  std::vector<float> left_arm_ang_vel(7, 0);
  left_arm_set[1] = -1.33;
  left_arm_set[2] = 0.3;
  left_arm_set[3] = -1.7;
  left_arm_set[4] = 1.2;
  left_arm_set[6] = 0.1;
  left_arm_msg = this->trajectory_generation.appendTrajectoryPoint(1.5, left_arm_set, left_arm_ang_vel, left_arm_msg);
  
  std::vector<float> right_arm_set(7, 0);
  std::vector<float> right_arm_ang_vel(7, 0);
  right_arm_set[1] = 1.33;
  right_arm_set[2] = 0.3;
  right_arm_set[3] = 1.7;
  right_arm_set[4] = 1.2;
  right_arm_set[6] = -0.1;
  right_arm_msg = this->trajectory_generation.appendTrajectoryPoint(1.5, right_arm_set, right_arm_ang_vel, right_arm_msg);
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.1;
  new_chest_orient[2] = yaw;
  float chest_time = 1.0;
  chest_msg.taskspace_trajectory_points.push_back(this->trajectory_generation.createSO3TrajectoryOrient(0.0, new_chest_orient, new_chest_ang_vel));
  
  pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  pelvis_msg.taskspace_trajectory_points.push_back(this->trajectory_generation.createPelvisHeightTrajectoryPoint(0.5, 1.06));
  
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
  neck_default[0] = 0.431;
  neck_default[1] = 0.0;
  neck_default[2] = -0.431;
  neck_msg= this->trajectory_generation.appendTrajectoryPoint(1.5, neck_default, neck_default_ang_vel, neck_msg);
  
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.5;
  finger_pos[1] = -0.6;
  finger_pos[2] = -0.2;
  finger_pos[3] = -0.2;
  finger_pos[4] = -0.2;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  finger_pos[0] = 0.5;
  finger_pos[1] = 0.6;
  finger_pos[2] = 0.2;
  finger_pos[3] = 0.2;
  finger_pos[4] = 0.2;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  
  full_msg output_msg;
  output_msg.trajectory_list_msg = trajectory_list_msg;
  output_msg.trajectory_neck_msg = neck_msg;
  output_msg.trajectory_finger_left_msg = left_fingers;
  output_msg.trajectory_finger_right_msg = right_fingers;
  output_msg.step_list_msg = step_list_msg;
  return output_msg;
}

std::vector<float>
Gait_Generation::getArmSwing(bool robot_side_arm, bool robot_side_step, float step_distance_x, float step_distance_y)
{
  std::vector<float> arm_swing(7, 0);
  
  if(robot_side_arm == LEFT)
  {
    if(robot_side_step == RIGHT)
    {
      arm_swing[0] = 0.0 + (-0.1 * step_distance_x/0.46);
      arm_swing[1] = -1.34;
      arm_swing[2] = 0.5;
      arm_swing[3] = -1.7 + (-0.1 * step_distance_x/0.46);
      arm_swing[4] = 1.2;
    }
    else
    {
      arm_swing[0] = 0.0 - (-0.1 * step_distance_x/0.46);
      arm_swing[1] = -1.47;
      arm_swing[2] = 0.2;
      arm_swing[3] = -1.7 - (-0.1 * step_distance_x/0.46);
      arm_swing[4] = 1.2;
    }
    
    if(step_distance_y > 0)
    {
      if(robot_side_step == LEFT)
      {
        arm_swing[1] += (0.1 * fabs(step_distance_y)/0.36);
        arm_swing[3] += (0.16 * fabs(step_distance_y)/0.36);
      }
      else if(robot_side_step == RIGHT)
      {
        arm_swing[1] += (0.12 * fabs(step_distance_y)/0.36);
      }
    }
    else if(step_distance_y < 0)
    {
      if(robot_side_step == RIGHT)
      {
        arm_swing[1] += (0.1 * fabs(step_distance_y)/0.36);
        arm_swing[3] += (0.1 * fabs(step_distance_y)/0.36);
      }
      else if(robot_side_step == LEFT)
      {
        arm_swing[1] += (0.12 * fabs(step_distance_y)/0.36);
      }
    }
  }
  else if(robot_side_arm == RIGHT)
  {
    if(robot_side_step == LEFT)
    {
      arm_swing[0] = 0.0 + (-0.1 * step_distance_x/0.46);
      arm_swing[1] = 1.34;
      arm_swing[2] = 0.5;
      arm_swing[3] = 1.7 - (-0.1 * step_distance_x/0.46);
      arm_swing[4] = 1.2;
    }
    else
    {
      arm_swing[0] = 0.0 - (-0.1 * step_distance_x/0.46);
      arm_swing[1] = 1.47;
      arm_swing[2] = 0.2;
      arm_swing[3] = 1.7 + (-0.1 * step_distance_x/0.46);
      arm_swing[4] = 1.2;
    }
      
    
    if(step_distance_y > 0)
    {
      if(robot_side_step == LEFT)
      {
        arm_swing[1] -= (0.1 * fabs(step_distance_y)/0.36);
        arm_swing[3] -= (0.1 * fabs(step_distance_y)/0.36);
      }
      else if(robot_side_step == RIGHT)
      {
        arm_swing[1] -= (0.08 * fabs(step_distance_y)/0.36);
      }
    }
    else if(step_distance_y < 0)
    {
      if(robot_side_step == RIGHT)
      {
        arm_swing[1] -= (0.2 * fabs(step_distance_y)/0.36);
        arm_swing[3] -= (0.16 * fabs(step_distance_y)/0.36);
      }
      else if(robot_side_step == LEFT)
      {
        arm_swing[1] -= (0.08 * fabs(step_distance_y)/0.36);
      }
    }
  }
  return arm_swing;
}

std::vector<float>
Gait_Generation::getArmAtRest(bool robot_side)
{
  std::vector<float> arm_at_rest(7, 0);
  
  if(robot_side == LEFT)
  {
    arm_at_rest[1] = -1.33;
    arm_at_rest[2] = 0.3;
    arm_at_rest[3] = -1.7;
    arm_at_rest[4] = 1.2;
  }
  else if(robot_side == RIGHT)
  {
    arm_at_rest[1] = 1.33;
    arm_at_rest[2] = 0.3;
    arm_at_rest[3] = 1.7;
    arm_at_rest[4] = 1.2;
  }
  
  return arm_at_rest;
}

std::vector<float>
Gait_Generation::getArmZeroAngVel()
{
  std::vector<float> arm_zero_ang_vel(7, 0);
  return arm_zero_ang_vel;
}

std::vector<float>
Gait_Generation::getChestZeroPos()
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
  
  std::vector<float> chest_orient (3, 0);
  chest_orient[2] = yaw;
  return chest_orient;
}

std::vector<float>
Gait_Generation::getChestSwing(bool robot_side_step, float step_distance_x, float step_distance_y)
{
  std::vector<float> chest_orient(3, 0);
  
  chest_orient[1] = (0.1 * step_distance_x/0.46);
  if(robot_side_step == LEFT)
  {
    chest_orient[0] = (-0.02 * step_distance_x/0.46);
    chest_orient[2] = (0.05 * step_distance_x/0.46);
    if(step_distance_y > 0)
    {
      chest_orient[0] += (-0.1 * step_distance_y/0.36);
    }
  }
  else if(robot_side_step == RIGHT)
  {
    chest_orient[0] = (0.02 * step_distance_x/0.46);
    chest_orient[2] = (-0.05 * step_distance_x/0.46);
    if(step_distance_y < 0)
    {
      chest_orient[0] += (-0.1 * step_distance_y/0.36);
    }
  }
  return chest_orient;
}

std::vector<float>
Gait_Generation::getChestSwing(bool robot_side_step, float step_distance_x, float step_distance_y, float heading)
{
  std::vector<float> chest_orient(3, 0);
  
  chest_orient[1] = (0.1 * step_distance_x/0.46);
  if(robot_side_step == LEFT)
  {
    chest_orient[0] = (-0.02 * step_distance_x/0.46);
    chest_orient[2] = (0.05 * step_distance_x/0.46);
    if(step_distance_y > 0)
    {
      chest_orient[0] += (-0.1 * step_distance_y/0.36);
    }
  }
  else if(robot_side_step == RIGHT)
  {
    chest_orient[0] = (0.02 * step_distance_x/0.46);
    chest_orient[2] = (-0.05 * step_distance_x/0.46);
    if(step_distance_y < 0)
    {
      chest_orient[0] += (-0.1 * step_distance_y/0.36);
    }
  }
  chest_orient[2] += heading;
  return chest_orient;
}

std::vector<float>
Gait_Generation::getChestZeroAngVel()
{
  std::vector<float> chest_orient (3, 0);
  return chest_orient;
}

ihmc_msgs::FootstepDataListRosMessage
Gait_Generation::createLowerBodyRotateGaitOverride(float heading)
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  //Sends walking commands to valkyrie
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  
  // Step list message configuration
  step_list_msg.execution_mode = ihmc_msgs::FootstepDataListRosMessage::OVERRIDE;
  step_list_msg.default_swing_time = DEFAULT_SWING_TIME;
  step_list_msg.default_transfer_time = DEFAULT_TRANSFER_TIME;
  step_list_msg.final_transfer_time = FINAL_TRANSFER_TIME;
  step_list_msg.unique_id = 1;
  
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
  
  tf2::Matrix3x3 rotation;
  yaw += heading;
  rotation.setRPY(0.0, 0.0, yaw);
  
  std::vector<float> new_step_pos_left (3, 0);
  new_step_pos_left[0] = 0.0;
  new_step_pos_left[1] = DEFAULT_STEP_SEPERATION/2;
  new_step_pos_left[2] = 0.0;
  std::vector<float> new_step_pos_right (3, 0);
  new_step_pos_right[0] = 0.0;
  new_step_pos_right[1] = -DEFAULT_STEP_SEPERATION/2;
  new_step_pos_right[2] = 0.0;
  
  x_vector new_step_pos;
  new_step_pos.x = new_step_pos_left[0]*rotation[0][0] + new_step_pos_left[1]*rotation[0][1] + new_step_pos_left[2]*rotation[0][2];
  new_step_pos.y = new_step_pos_left[0]*rotation[1][0] + new_step_pos_left[1]*rotation[1][1] + new_step_pos_left[2]*rotation[1][2];
  new_step_pos.z = new_step_pos_left[0]*rotation[2][0] + new_step_pos_left[1]*rotation[2][1] + new_step_pos_left[2]*rotation[2][2];
  new_step_pos_left[0] = new_step_pos.x;
  new_step_pos_left[1] = new_step_pos.y;
  new_step_pos_left[2] = new_step_pos.z;
  new_step_pos.x = new_step_pos_right[0]*rotation[0][0] + new_step_pos_right[1]*rotation[0][1] + new_step_pos_right[2]*rotation[0][2];
  new_step_pos.y = new_step_pos_right[0]*rotation[1][0] + new_step_pos_right[1]*rotation[1][1] + new_step_pos_right[2]*rotation[1][2];
  new_step_pos.z = new_step_pos_right[0]*rotation[2][0] + new_step_pos_right[1]*rotation[2][1] + new_step_pos_right[2]*rotation[2][2];
  new_step_pos_right[0] = new_step_pos.x;
  new_step_pos_right[1] = new_step_pos.y;
  new_step_pos_right[2] = new_step_pos.z;
  
  if(heading < 0)
  {
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetReset(RIGHT, 0.0, new_step_pos_right, heading));
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetReset(LEFT, 0.0, new_step_pos_left, heading));
  }
  else if(heading > 0)
  {
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetReset(LEFT, 0.0, new_step_pos_left, heading));
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetReset(RIGHT, 0.0, new_step_pos_right, heading));
  }
  
  return step_list_msg;
}

// Override generators
ihmc_msgs::FootstepDataListRosMessage
Gait_Generation::createLowerBodyLinearGaitOverride(x_vector destination, float heading)
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
  //Sends walking commands to valkyrie
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  
  // Step list message configuration
  step_list_msg.execution_mode = ihmc_msgs::FootstepDataListRosMessage::OVERRIDE;
  step_list_msg.default_swing_time = DEFAULT_SWING_TIME;
  step_list_msg.default_transfer_time = DEFAULT_TRANSFER_TIME;
  step_list_msg.final_transfer_time = FINAL_TRANSFER_TIME;
  step_list_msg.unique_id = 1;

  const float combined_step_distance = DEFAULT_STEP_DISTANCE;
  const float foot_seperation = DEFAULT_STEP_SEPERATION;
  const float conversion_factor = combined_step_distance/sqrt(pow(destination.x, 2.0) + pow(destination.y, 2.0));
  const float step_distance_x = destination.x * conversion_factor;
  const float step_distance_y = destination.y * conversion_factor;
  const float step_distance_z = destination.z * conversion_factor;
  ROS_INFO("Setup: %f", conversion_factor);
  
  std::vector<float> new_step_pos(3, 0);
  std::vector<float> new_step_pos_temp(3, 0);
  new_step_pos[2] = DEFAULT_STEP_HEIGHT;
  
  geometry_msgs::TransformStamped torso = tfBuffer.lookupTransform(WORLD, TORSO, ros::Time(0), ros::Duration(0.4));
  
  tf2::Quaternion last_known_heading(torso.transform.rotation.x, torso.transform.rotation.y, torso.transform.rotation.z, torso.transform.rotation.w);
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Matrix3x3 rotation(last_known_heading);
  rotation.getRPY(roll, pitch, yaw);

  if(fabs(destination.x)/4.0 >= fabs(destination.y))
  {
    bool robot_side = LEFT;
    while(fabs(new_step_pos[0]) <= fabs(destination.x) - 0.01)
    {
      if(fabs(destination.x) - fabs(new_step_pos[0]) < fabs(step_distance_x))
      {
        new_step_pos[0] = destination.x;
        new_step_pos[1] = destination.y;
        new_step_pos[2] = destination.z;
      }
      else
      {
        new_step_pos[0] += step_distance_x;
        new_step_pos[1] += step_distance_y;
        new_step_pos[2] += step_distance_z;
      }
      
      std::vector<float> temp_pos(3, 0);
      temp_pos[0] = new_step_pos[0];
      temp_pos[1] = new_step_pos[1];
      temp_pos[2] = new_step_pos[2];
      ROS_INFO("Distance: %f", temp_pos[0]);
      
      tf2::Matrix3x3 orientation;
      orientation.setRPY(0.0, 0.0, heading + yaw);
      new_step_pos_temp[0] = temp_pos[0]*orientation[0][0] + temp_pos[1]*orientation[0][1] + temp_pos[2]*orientation[0][2];
      new_step_pos_temp[1] = temp_pos[0]*orientation[1][0] + temp_pos[1]*orientation[1][1] + temp_pos[2]*orientation[1][2];
      new_step_pos_temp[2] = temp_pos[0]*orientation[2][0] + temp_pos[1]*orientation[2][1] + temp_pos[2]*orientation[2][2];
      
      if(robot_side == LEFT)
      {
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
        robot_side = RIGHT;
      }
      else if(robot_side == RIGHT)
      {
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
        robot_side = LEFT;
      }
    }
  }
  else
  {
    bool robot_side;
    if(destination.y > 0)
    {
      robot_side = LEFT;
    }
    else if(destination.y < 0)
    {
      robot_side = RIGHT;
    }
    
    while(fabs(new_step_pos[1]) <= fabs(destination.y) - 0.01)
    {
      if(fabs(destination.y) - fabs(new_step_pos[1]) < fabs(step_distance_y))
      {
        new_step_pos[0] = destination.x;
        new_step_pos[1] = destination.y;
        new_step_pos[2] = destination.z;
      }
      else
      {
        new_step_pos[0] += step_distance_x;
        new_step_pos[1] += step_distance_y;
        new_step_pos[2] += step_distance_z;
      }
      
      std::vector<float> temp_pos(3, 0);
      temp_pos[0] = new_step_pos[0];
      temp_pos[1] = new_step_pos[1];
      temp_pos[2] = new_step_pos[2];
      if(robot_side == LEFT)
      {
        temp_pos[1] += foot_seperation;
      }
      else if(robot_side == RIGHT)
      {
        temp_pos[1] -= foot_seperation;
      }
      tf2::Matrix3x3 orientation;
      orientation.setRPY(0.0, 0.0, heading + yaw);
      new_step_pos_temp[0] = temp_pos[0]*orientation[0][0] + temp_pos[1]*orientation[0][1] + temp_pos[2]*orientation[0][2];
      new_step_pos_temp[1] = temp_pos[0]*orientation[1][0] + temp_pos[1]*orientation[1][1] + temp_pos[2]*orientation[1][2];
      new_step_pos_temp[2] = temp_pos[0]*orientation[2][0] + temp_pos[1]*orientation[2][1] + temp_pos[2]*orientation[2][2];
      
      if(robot_side == LEFT)
      {
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
      }
      else if(robot_side == RIGHT)
      {
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
      }
    }
  }
  return step_list_msg;
}

ihmc_msgs::WholeBodyTrajectoryRosMessage
Gait_Generation::createUpperBodyGait(ihmc_msgs::FootstepDataListRosMessage step_list)
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
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  
  const float time_interval = step_list.default_swing_time + (step_list.default_transfer_time*1.0);
  float time_of_execution = time_interval - 0.2;
  
  geometry_msgs::Vector3 starting_right_foot = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2)).transform.translation;
  geometry_msgs::Vector3 starting_left_foot = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2)).transform.translation;
  
  //chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(time_of_execution, getChestZeroPos(), getChestZeroAngVel()));
  left_arm_msg = this->trajectory_generation.appendTrajectoryPoint(time_of_execution, getArmAtRest(LEFT), getArmZeroAngVel(), left_arm_msg);
  right_arm_msg = this->trajectory_generation.appendTrajectoryPoint(time_of_execution, getArmAtRest(RIGHT), getArmZeroAngVel(), right_arm_msg);
  time_of_execution += time_interval;
  
  const int num_steps = step_list.footstep_data_list.size();
  for(int step_id = 0; step_id < num_steps-1; step_id++)
  {
    float step_distance_x;
    float step_distance_y;
    tf2Scalar x = step_list.footstep_data_list[step_id].orientation.x;
    tf2Scalar y = step_list.footstep_data_list[step_id].orientation.y;
    tf2Scalar z = step_list.footstep_data_list[step_id].orientation.z;
    tf2Scalar w = step_list.footstep_data_list[step_id].orientation.w;
    
    tf2::Quaternion heading(x, y, z, w);
    tf2::Matrix3x3 orient_fix(heading);
    tf2Scalar roll;
    tf2Scalar pitch;
    tf2Scalar yaw;
    orient_fix.getRPY(roll, pitch, yaw);
    if(step_list.footstep_data_list[step_id].robot_side == ihmc_msgs::FootstepDataRosMessage::RIGHT)
    {
      orient_fix.setRPY(0.0, 0.0, -yaw + 0.06);
      
      if(step_id == 0)
      {
        x_vector current_foot;
        current_foot.x = step_list.footstep_data_list[step_id].location.x*orient_fix[0][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[0][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[0][2];
        current_foot.y = step_list.footstep_data_list[step_id].location.x*orient_fix[1][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[1][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[1][2];
        current_foot.z = step_list.footstep_data_list[step_id].location.x*orient_fix[2][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[2][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[2][2];
        x_vector previous_foot;
        previous_foot.x = starting_right_foot.x*orient_fix[0][0] + starting_right_foot.y*orient_fix[0][1] + starting_right_foot.z*orient_fix[0][2];
        previous_foot.y = starting_right_foot.x*orient_fix[1][0] + starting_right_foot.y*orient_fix[1][1] + starting_right_foot.z*orient_fix[1][2];
        previous_foot.z = starting_right_foot.x*orient_fix[2][0] + starting_right_foot.y*orient_fix[2][1] + starting_right_foot.z*orient_fix[2][2];
        
        step_distance_x = current_foot.x - previous_foot.x;
        step_distance_y = current_foot.y - previous_foot.y;
      }
      else
      {
        x_vector current_foot;
        current_foot.x = step_list.footstep_data_list[step_id].location.x*orient_fix[0][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[0][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[0][2];
        current_foot.y = step_list.footstep_data_list[step_id].location.x*orient_fix[1][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[1][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[1][2];
        current_foot.z = step_list.footstep_data_list[step_id].location.x*orient_fix[2][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[2][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[2][2];
        x_vector previous_foot;
        previous_foot.x = step_list.footstep_data_list[step_id-1].location.x*orient_fix[0][0] + step_list.footstep_data_list[step_id-1].location.y*orient_fix[0][1] + step_list.footstep_data_list[step_id-1].location.z*orient_fix[0][2];
        previous_foot.y = step_list.footstep_data_list[step_id-1].location.x*orient_fix[1][0] + step_list.footstep_data_list[step_id-1].location.y*orient_fix[1][1] + step_list.footstep_data_list[step_id-1].location.z*orient_fix[1][2];
        previous_foot.z = step_list.footstep_data_list[step_id-1].location.x*orient_fix[2][0] + step_list.footstep_data_list[step_id-1].location.y*orient_fix[2][1] + step_list.footstep_data_list[step_id-1].location.z*orient_fix[2][2];
        
        step_distance_x = current_foot.x - previous_foot.x;
        step_distance_y = current_foot.y - previous_foot.y;
      }
      
      std::vector<float> left_arm_swing = getArmSwing(LEFT, RIGHT, step_distance_x, step_distance_y);
      std::vector<float> right_arm_swing = getArmSwing(RIGHT, RIGHT, step_distance_x, step_distance_y);
      //chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(time_of_execution, getChestSwing(RIGHT, step_distance_x, step_distance_y, yaw - 0.06), getChestZeroAngVel()));
      left_arm_msg = this->trajectory_generation.appendTrajectoryPoint(time_of_execution, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
      right_arm_msg = this->trajectory_generation.appendTrajectoryPoint(time_of_execution, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
    }
    else if(step_list.footstep_data_list[step_id].robot_side == ihmc_msgs::FootstepDataRosMessage::LEFT)
    {
      orient_fix.setRPY(0.0, 0.0, -(yaw - 0.06));
      
      if(step_id == 0)
      {
        x_vector current_foot;
        current_foot.x = step_list.footstep_data_list[step_id].location.x*orient_fix[0][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[0][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[0][2];
        current_foot.y = step_list.footstep_data_list[step_id].location.x*orient_fix[1][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[1][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[1][2];
        current_foot.z = step_list.footstep_data_list[step_id].location.x*orient_fix[2][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[2][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[2][2];
        x_vector previous_foot;
        previous_foot.x = starting_left_foot.x*orient_fix[0][0] + starting_left_foot.y*orient_fix[0][1] + starting_left_foot.z*orient_fix[0][2];
        previous_foot.y = starting_left_foot.x*orient_fix[1][0] + starting_left_foot.y*orient_fix[1][1] + starting_left_foot.z*orient_fix[1][2];
        previous_foot.z = starting_left_foot.x*orient_fix[2][0] + starting_left_foot.y*orient_fix[2][1] + starting_left_foot.z*orient_fix[2][2];
        
        step_distance_x = current_foot.x - previous_foot.x;
        step_distance_y = current_foot.y - previous_foot.y;
      }
      else
      {
        x_vector current_foot;
        current_foot.x = step_list.footstep_data_list[step_id].location.x*orient_fix[0][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[0][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[0][2];
        current_foot.y = step_list.footstep_data_list[step_id].location.x*orient_fix[1][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[1][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[1][2];
        current_foot.z = step_list.footstep_data_list[step_id].location.x*orient_fix[2][0] + step_list.footstep_data_list[step_id].location.y*orient_fix[2][1] + step_list.footstep_data_list[step_id].location.z*orient_fix[2][2];
        x_vector previous_foot;
        previous_foot.x = step_list.footstep_data_list[step_id-1].location.x*orient_fix[0][0] + step_list.footstep_data_list[step_id-1].location.y*orient_fix[0][1] + step_list.footstep_data_list[step_id-1].location.z*orient_fix[0][2];
        previous_foot.y = step_list.footstep_data_list[step_id-1].location.x*orient_fix[1][0] + step_list.footstep_data_list[step_id-1].location.y*orient_fix[1][1] + step_list.footstep_data_list[step_id-1].location.z*orient_fix[1][2];
        previous_foot.z = step_list.footstep_data_list[step_id-1].location.x*orient_fix[2][0] + step_list.footstep_data_list[step_id-1].location.y*orient_fix[2][1] + step_list.footstep_data_list[step_id-1].location.z*orient_fix[2][2];
        
        step_distance_x = current_foot.x - previous_foot.x;
        step_distance_y = current_foot.y - previous_foot.y;
      }
      
      std::vector<float> left_arm_swing = getArmSwing(LEFT, LEFT, step_distance_x, step_distance_y);
      std::vector<float> right_arm_swing = getArmSwing(RIGHT, LEFT, step_distance_x, step_distance_y);
      //chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(time_of_execution, getChestSwing(LEFT, step_distance_x, step_distance_y, yaw - 0.06), getChestZeroAngVel()));
      left_arm_msg = this->trajectory_generation.appendTrajectoryPoint(time_of_execution, left_arm_swing, getArmZeroAngVel(), left_arm_msg);
      right_arm_msg = this->trajectory_generation.appendTrajectoryPoint(time_of_execution, right_arm_swing, getArmZeroAngVel(), right_arm_msg);
    }
    time_of_execution += time_interval;
  }
  //chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(time_of_execution, getChestZeroAngVel(), getChestZeroAngVel()));
  left_arm_msg = this->trajectory_generation.appendTrajectoryPoint(time_of_execution, getArmAtRest(LEFT), getArmZeroAngVel(), left_arm_msg);
  right_arm_msg = this->trajectory_generation.appendTrajectoryPoint(time_of_execution, getArmAtRest(RIGHT), getArmZeroAngVel(), right_arm_msg);
  
  // Compress trajectories into one message
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  return trajectory_list_msg;
}

ihmc_msgs::FootstepDataListRosMessage
Gait_Generation::createLowerBodyArcGaitOverride(bool turn, float arc_length, float radian)
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  //Sends walking commands to valkyrie
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  
  step_list_msg.execution_mode = ihmc_msgs::FootstepDataListRosMessage::OVERRIDE;
  step_list_msg.default_swing_time = DEFAULT_SWING_TIME;
  step_list_msg.default_transfer_time = DEFAULT_TRANSFER_TIME;
  step_list_msg.final_transfer_time = FINAL_TRANSFER_TIME;
  step_list_msg.unique_id = 1;
  
  float step_distance = DEFAULT_STEP_DISTANCE;
  float foot_seperation = DEFAULT_STEP_SEPERATION;
  std::vector<float> new_step_pos(3, 0);
  std::vector<float> new_step_pos_temp(3, 0);
  
  // Pre calculates math for arc paths
  float num_steps = arc_length/step_distance;
  if(step_distance > arc_length)
  {
    num_steps = 4.0;
  }
  float radian_increment = radian/num_steps;
  foot_seperation += (0.2 * fabs(radian_increment)/M_PI);
  float radius = arc_length/radian;
  float radius_left = radius - foot_seperation/2.0;
  float radius_right = radius + foot_seperation/2.0;
  float current_radian_increment = 0.0;
  float foot_orient = current_radian_increment;
  
  // Switches direction of arc
  if(turn == RIGHT)
  {
    radius_left = radius + foot_seperation/2.0;
    radius_right = radius - foot_seperation/2.0;
  }
  
  geometry_msgs::TransformStamped torso = tfBuffer.lookupTransform(WORLD, TORSO, ros::Time(0), ros::Duration(0.4));
  
  tf2::Quaternion heading(torso.transform.rotation.x, torso.transform.rotation.y, torso.transform.rotation.z, torso.transform.rotation.w);
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Matrix3x3 last_heading(heading);
  last_heading.getRPY(roll, pitch, yaw);
  last_heading.setRPY(0.0, 0.0, yaw);
  
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  
  x_vector left_foot;
  x_vector right_foot;
  
  tf2::Matrix3x3 inverse_rotation(last_heading.inverse());
  left_foot.x = left_foot_ankle.transform.translation.x*inverse_rotation[0][0] + left_foot_ankle.transform.translation.y*inverse_rotation[0][1] + left_foot_ankle.transform.translation.z*inverse_rotation[0][2];
  left_foot.y = left_foot_ankle.transform.translation.x*inverse_rotation[1][0] + left_foot_ankle.transform.translation.y*inverse_rotation[1][1] + left_foot_ankle.transform.translation.z*inverse_rotation[1][2];
  left_foot.z = left_foot_ankle.transform.translation.x*inverse_rotation[2][0] + left_foot_ankle.transform.translation.y*inverse_rotation[2][1] + left_foot_ankle.transform.translation.z*inverse_rotation[2][2];
  right_foot.x = right_foot_ankle.transform.translation.x*inverse_rotation[0][0] + right_foot_ankle.transform.translation.y*inverse_rotation[0][1] + right_foot_ankle.transform.translation.z*inverse_rotation[0][2];
  right_foot.y = right_foot_ankle.transform.translation.x*inverse_rotation[1][0] + right_foot_ankle.transform.translation.y*inverse_rotation[1][1] + right_foot_ankle.transform.translation.z*inverse_rotation[1][2];
  right_foot.z = right_foot_ankle.transform.translation.x*inverse_rotation[2][0] + right_foot_ankle.transform.translation.y*inverse_rotation[2][1] + right_foot_ankle.transform.translation.z*inverse_rotation[2][2];
  bool robot_side = LEFT;
  if(left_foot.x > right_foot.x)
  {
    robot_side = RIGHT;
  }
  if(left_foot.x - right_foot.x > step_distance/2 || right_foot.x - left_foot.x > step_distance/2)
  {
    current_radian_increment += radian_increment;
  }
  
  while(current_radian_increment < radian)
  {
    new_step_pos[2] = DEFAULT_STEP_HEIGHT;
    if(current_radian_increment > radian - radian_increment)
    {
      current_radian_increment = radian;
    }
    else
    {
      current_radian_increment += radian_increment;
    }
    if(robot_side == LEFT)
    {
      new_step_pos[0] = radius_left * sin(current_radian_increment);
      if((current_radian_increment > M_PI/2 && current_radian_increment < M_PI*3.0/2.0) || (current_radian_increment < -M_PI/2 && current_radian_increment > -M_PI* 3.0/2.0))
      {
        new_step_pos[1] = -(-sqrt(pow(radius_left, 2.0) - pow(new_step_pos[0], 2.0)) - radius_left);
      }
      else
      {
        new_step_pos[1] = -(sqrt(pow(radius_left, 2.0) - pow(new_step_pos[0], 2.0)) - radius_left);
      }
      foot_orient = current_radian_increment;
      
      if(turn == RIGHT)
      {
        new_step_pos[1] = -new_step_pos[1];
        foot_orient = -foot_orient;
      }
      
      new_step_pos_temp[0] = new_step_pos[0]*last_heading[0][0] + new_step_pos[1]*last_heading[0][1] + new_step_pos[2]*last_heading[0][2];
      new_step_pos_temp[1] = new_step_pos[0]*last_heading[1][0] + new_step_pos[1]*last_heading[1][1] + new_step_pos[2]*last_heading[1][2];
      new_step_pos_temp[2] = new_step_pos[0]*last_heading[2][0] + new_step_pos[1]*last_heading[2][1] + new_step_pos[2]*last_heading[2][2];
      new_step_pos[0] = new_step_pos_temp[0];
      new_step_pos[1] = new_step_pos_temp[1];
      new_step_pos[2] = new_step_pos_temp[2];
      ROS_INFO("Yaw: %f", foot_orient + yaw);
      
      step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetArc(LEFT, step_distance, new_step_pos, foot_orient + yaw));
      robot_side = RIGHT;
    }
    else
    {
      new_step_pos[0] = radius_right * sin(current_radian_increment);
      if((current_radian_increment > M_PI/2 && current_radian_increment < M_PI*3.0/2.0) || (current_radian_increment < -M_PI/2 && current_radian_increment > -M_PI* 3.0/2.0))
      {
        new_step_pos[1] = -(-sqrt(pow(radius_right, 2.0) - pow(new_step_pos[0], 2.0)) - radius_right);
      }
      else
      {
        new_step_pos[1] = -(sqrt(pow(radius_right, 2.0) - pow(new_step_pos[0], 2.0)) - radius_right);
      }
      foot_orient = current_radian_increment;
      
      if(turn == RIGHT)
      {
        new_step_pos[1] = -new_step_pos[1];
        foot_orient = -foot_orient;
      }
      
      new_step_pos_temp[0] = new_step_pos[0]*last_heading[0][0] + new_step_pos[1]*last_heading[0][1] + new_step_pos[2]*last_heading[0][2];
      new_step_pos_temp[1] = new_step_pos[0]*last_heading[1][0] + new_step_pos[1]*last_heading[1][1] + new_step_pos[2]*last_heading[1][2];
      new_step_pos_temp[2] = new_step_pos[0]*last_heading[2][0] + new_step_pos[1]*last_heading[2][1] + new_step_pos[2]*last_heading[2][2];
      new_step_pos[0] = new_step_pos_temp[0];
      new_step_pos[1] = new_step_pos_temp[1];
      new_step_pos[2] = new_step_pos_temp[2];
      ROS_INFO("Yaw: %f", foot_orient + yaw);
      
      step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetArc(RIGHT, step_distance, new_step_pos, foot_orient + yaw));
      robot_side = LEFT;
    }
  }
  ROS_INFO("Arc End\n");
  return step_list_msg;
}

ihmc_msgs::FootstepDataListRosMessage
Gait_Generation::createLowerBodyRotateGaitQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, float heading)
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
  // Retain previous ending point which would be the starting point
  int last_known_msg_id = step_list_msg.footstep_data_list.size() - 1;
  bool last_known_robot_side = step_list_msg.footstep_data_list[last_known_msg_id].robot_side;
  float last_step_offset;
  std::vector<float> last_known_pos(3, 0);
  std::vector<float> scnd_last_known_pos(3, 0);
  std::vector<float> starting_pos_offset(3, 0);
  last_known_pos[0] = step_list_msg.footstep_data_list[last_known_msg_id].location.x;
  last_known_pos[1] = step_list_msg.footstep_data_list[last_known_msg_id].location.y;
  last_known_pos[2] = step_list_msg.footstep_data_list[last_known_msg_id].location.z;
  
  
  std::vector<float> last_heading_temp(4,0);
  last_heading_temp[0] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.x;
  last_heading_temp[1] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.y;
  last_heading_temp[2] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.z;
  last_heading_temp[3] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.w;
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Quaternion last_known_heading(last_heading_temp[0],last_heading_temp[1],last_heading_temp[2],last_heading_temp[3]);
  tf2::Matrix3x3 last_heading(last_known_heading);
  last_heading.getRPY(roll, pitch, yaw);
  std::vector<float> new_step_seperation(3, 0);
  std::vector<float> step_seperation(3, 0);
  // Utilize previous msg to complete necessary information
  if(last_known_robot_side == LEFT)
  {
    yaw -= 0.06;
    new_step_seperation[1] = -DEFAULT_STEP_SEPERATION/2;
  }
  else if(last_known_robot_side == RIGHT)
  {
    yaw += 0.06;
    new_step_seperation[1] = DEFAULT_STEP_SEPERATION/2;
  }
  last_heading.setRPY(0.0, 0.0, yaw);
  last_known_pos[0] += new_step_seperation[0]*last_heading[0][0] + new_step_seperation[1]*last_heading[0][1] + new_step_seperation[2]*last_heading[0][2];
  last_known_pos[1] += new_step_seperation[0]*last_heading[1][0] + new_step_seperation[1]*last_heading[1][1] + new_step_seperation[2]*last_heading[1][2];
  last_known_pos[2] += new_step_seperation[0]*last_heading[2][0] + new_step_seperation[1]*last_heading[2][1] + new_step_seperation[2]*last_heading[2][2];
  yaw += heading;
  last_heading.setRPY(0.0, 0.0, yaw);
  
  std::vector<float> new_step_pos_left (3, 0);
  new_step_pos_left[0] = 0.0;
  new_step_pos_left[1] = DEFAULT_STEP_SEPERATION/2;
  new_step_pos_left[2] = 0.0;
  std::vector<float> new_step_pos_right (3, 0);
  new_step_pos_right[0] = 0.0;
  new_step_pos_right[1] = -DEFAULT_STEP_SEPERATION/2;
  new_step_pos_right[2] = 0.0;
  
  x_vector new_step_pos;
  new_step_pos.x = new_step_pos_left[0]*last_heading[0][0] + new_step_pos_left[1]*last_heading[0][1] + new_step_pos_left[2]*last_heading[0][2];
  new_step_pos.y = new_step_pos_left[0]*last_heading[1][0] + new_step_pos_left[1]*last_heading[1][1] + new_step_pos_left[2]*last_heading[1][2];
  new_step_pos.z = new_step_pos_left[0]*last_heading[2][0] + new_step_pos_left[1]*last_heading[2][1] + new_step_pos_left[2]*last_heading[2][2];
  new_step_pos_left[0] = new_step_pos.x + last_known_pos[0];
  new_step_pos_left[1] = new_step_pos.y + last_known_pos[1];
  new_step_pos_left[2] = new_step_pos.z + last_known_pos[2];
  new_step_pos.x = new_step_pos_right[0]*last_heading[0][0] + new_step_pos_right[1]*last_heading[0][1] + new_step_pos_right[2]*last_heading[0][2];
  new_step_pos.y = new_step_pos_right[0]*last_heading[1][0] + new_step_pos_right[1]*last_heading[1][1] + new_step_pos_right[2]*last_heading[1][2];
  new_step_pos.z = new_step_pos_right[0]*last_heading[2][0] + new_step_pos_right[1]*last_heading[2][1] + new_step_pos_right[2]*last_heading[2][2];
  new_step_pos_right[0] = new_step_pos.x + last_known_pos[0];
  new_step_pos_right[1] = new_step_pos.y + last_known_pos[1];
  new_step_pos_right[2] = new_step_pos.z + last_known_pos[2];
  
  if(heading < 0)
  {
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(RIGHT, 0.0, new_step_pos_right, yaw));
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, 0.0, new_step_pos_left, yaw));
  }
  else if(heading > 0)
  {
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, 0.0, new_step_pos_left, yaw));
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(RIGHT, 0.0, new_step_pos_right, yaw));
  }
  
  
  return step_list_msg;
  
}

// Queue generators
ihmc_msgs::FootstepDataListRosMessage
Gait_Generation::createLowerBodyLinearGaitQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, x_vector destination, float heading)
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  // Retain previous ending point which would be the starting point
  int last_known_msg_id = step_list_msg.footstep_data_list.size() - 1;
  bool last_known_robot_side = step_list_msg.footstep_data_list[last_known_msg_id].robot_side;
  float last_step_offset = 0;
  std::vector<float> last_known_pos(3, 0);
  std::vector<float> scnd_last_known_pos(3, 0);
  std::vector<float> starting_pos_offset(3, 0);
  last_known_pos[0] = step_list_msg.footstep_data_list[last_known_msg_id].location.x;
  last_known_pos[1] = step_list_msg.footstep_data_list[last_known_msg_id].location.y;
  last_known_pos[2] = step_list_msg.footstep_data_list[last_known_msg_id].location.z;
  ROS_INFO("LASTXYZ:\t%f\t%f\t%f", last_known_pos[0], last_known_pos[1], last_known_pos[2]);
  scnd_last_known_pos[0] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.x;
  scnd_last_known_pos[1] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.y;
  scnd_last_known_pos[2] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.z;
  starting_pos_offset[0] = last_known_pos[0] - scnd_last_known_pos[0];
  starting_pos_offset[1] = last_known_pos[1] - scnd_last_known_pos[1];
  starting_pos_offset[2] = last_known_pos[2] - scnd_last_known_pos[2];
  
  std::vector<float> last_heading_temp(4,0);
  last_heading_temp[0] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.x;
  last_heading_temp[1] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.y;
  last_heading_temp[2] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.z;
  last_heading_temp[3] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.w;
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Quaternion last_known_heading(last_heading_temp[0],last_heading_temp[1],last_heading_temp[2],last_heading_temp[3]);
  tf2::Matrix3x3 last_heading(last_known_heading);
  last_heading.getRPY(roll, pitch, yaw);
  std::vector<float> new_step_seperation(3, 0);
  std::vector<float> step_seperation(3, 0);
  // Utilize previous msg to complete necessary information
  if(last_known_robot_side == LEFT)
  {
    yaw -= 0.06;
    new_step_seperation[1] = -DEFAULT_STEP_SEPERATION/2;
  }
  else if(last_known_robot_side == RIGHT)
  {
    yaw += 0.06;
    new_step_seperation[1] = DEFAULT_STEP_SEPERATION/2;
  }
  last_heading.setRPY(0.0, 0.0, yaw);
  step_seperation[0] = new_step_seperation[0]*last_heading[0][0] + new_step_seperation[1]*last_heading[0][1] + new_step_seperation[2]*last_heading[0][2];
  step_seperation[1] = new_step_seperation[0]*last_heading[1][0] + new_step_seperation[1]*last_heading[1][1] + new_step_seperation[2]*last_heading[1][2];
  step_seperation[2] = new_step_seperation[0]*last_heading[2][0] + new_step_seperation[1]*last_heading[2][1] + new_step_seperation[2]*last_heading[2][2];
  tf2::Matrix3x3 inverse_heading(last_heading.inverse());
  last_step_offset += starting_pos_offset[0]*inverse_heading[0][0] + starting_pos_offset[1]*inverse_heading[0][1] + starting_pos_offset[2]*inverse_heading[0][2];
  
  destination.x -= (last_known_pos[0] + step_seperation[0]);
  destination.y -= (last_known_pos[1] + step_seperation[1]);
  destination.z -= (last_known_pos[2] + step_seperation[2]);
  step_seperation[0] = destination.x;
  step_seperation[1] = destination.y;
  step_seperation[2] = destination.z;
  destination.x = step_seperation[0]*inverse_heading[0][0] + step_seperation[1]*inverse_heading[0][1] + step_seperation[2]*inverse_heading[0][2];
  destination.y = step_seperation[0]*inverse_heading[1][0] + step_seperation[1]*inverse_heading[1][1] + step_seperation[2]*inverse_heading[1][2];
  destination.z = step_seperation[0]*inverse_heading[2][0] + step_seperation[1]*inverse_heading[2][1] + step_seperation[2]*inverse_heading[2][2];
  
  if(last_known_robot_side == LEFT)
  {
    new_step_seperation[1] = -DEFAULT_STEP_SEPERATION;
  }
  else if(last_known_robot_side == RIGHT)
  {
    new_step_seperation[1] = DEFAULT_STEP_SEPERATION;
  }
  step_seperation[0] = new_step_seperation[0]*last_heading[0][0] + new_step_seperation[1]*last_heading[0][1] + new_step_seperation[2]*last_heading[0][2];
  step_seperation[1] = new_step_seperation[0]*last_heading[1][0] + new_step_seperation[1]*last_heading[1][1] + new_step_seperation[2]*last_heading[1][2];
  step_seperation[2] = new_step_seperation[0]*last_heading[2][0] + new_step_seperation[1]*last_heading[2][1] + new_step_seperation[2]*last_heading[2][2];
  
  // Set constants for controlling linear gait
  const float combined_step_distance = DEFAULT_STEP_DISTANCE;
  const float foot_seperation = DEFAULT_STEP_SEPERATION;
  const float step_height = DEFAULT_STEP_HEIGHT;
  const float conversion_factor = combined_step_distance/sqrt(pow(destination.x, 2.0) + pow(destination.y, 2.0));
  const float step_distance_x = destination.x * conversion_factor;
  const float step_distance_y = destination.y * conversion_factor;
  const float step_distance_z = destination.z * conversion_factor;
  bool robot_side = last_known_robot_side;
  std::vector<float> new_step_pos(3, 0);
  std::vector<float> new_step_pos_temp(3, 0);
  
  // Set destination with one step forward because of queueing nature
  // Also add step height offset for walking stabalization
  new_step_pos[0] = -last_step_offset;
  
  step_list_msg.footstep_data_list.pop_back();

  if(fabs(destination.x)/4.0 >= fabs(destination.y))
  {
    while(fabs(new_step_pos[0]) <= fabs(destination.x) - 0.01)
    {
      if(fabs(destination.x) - fabs(new_step_pos[0]) < step_distance_x)
      {
        new_step_pos[0] = destination.x;
        new_step_pos[1] = destination.y;
        new_step_pos[2] = destination.z;
      }
      else
      {
        new_step_pos[0] += step_distance_x;
        new_step_pos[1] += step_distance_y;
        new_step_pos[2] += step_distance_z;
      }
      new_step_pos[2] = DEFAULT_STEP_HEIGHT;
      tf2::Matrix3x3 rotation;
      rotation.setRPY(0.0, 0.0, heading + yaw);
      new_step_pos_temp[0] = new_step_pos[0]*rotation[0][0] + new_step_pos[1]*rotation[0][1] + new_step_pos[2]*rotation[0][2];
      new_step_pos_temp[1] = new_step_pos[0]*rotation[1][0] + new_step_pos[1]*rotation[1][1] + new_step_pos[2]*rotation[1][2];
      new_step_pos_temp[2] = new_step_pos[0]*rotation[2][0] + new_step_pos[1]*rotation[2][1] + new_step_pos[2]*rotation[2][2];
      new_step_pos_temp[0] += last_known_pos[0];
      new_step_pos_temp[1] += last_known_pos[1];
      new_step_pos_temp[2] += last_known_pos[2];
      
      if(robot_side == LEFT)
      {
        if(last_known_robot_side == RIGHT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
        }
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
        robot_side = RIGHT;
      }
      else if(robot_side == RIGHT)
      {
        if(last_known_robot_side == LEFT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
        }
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
        robot_side = LEFT;
      }
    }
  }
  else
  {
    while(fabs(new_step_pos[1]) <= fabs(destination.y) - 0.01)
    {
      if(fabs(destination.y) - fabs(new_step_pos[1]) < step_distance_y)
      {
        new_step_pos[0] = destination.x;
        new_step_pos[1] = destination.y;
        new_step_pos[2] = destination.z;
      }
      else
      {
        new_step_pos[0] += step_distance_x;
        new_step_pos[1] += step_distance_y;
        new_step_pos[2] += step_distance_z;
      }
      new_step_pos[2] = DEFAULT_STEP_HEIGHT;
      tf2::Matrix3x3 rotation;
      rotation.setRPY(0.0, 0.0, heading + yaw);
      new_step_pos_temp[0] = new_step_pos[0]*rotation[0][0] + new_step_pos[1]*rotation[0][1] + new_step_pos[2]*rotation[0][2];
      new_step_pos_temp[1] = new_step_pos[0]*rotation[1][0] + new_step_pos[1]*rotation[1][1] + new_step_pos[2]*rotation[1][2];
      new_step_pos_temp[2] = new_step_pos[0]*rotation[2][0] + new_step_pos[1]*rotation[2][1] + new_step_pos[2]*rotation[2][2];
      new_step_pos_temp[0] += last_known_pos[0];
      new_step_pos_temp[1] += last_known_pos[1];
      new_step_pos_temp[2] += last_known_pos[2];
      
      if(robot_side == LEFT)
      {
        if(last_known_robot_side == RIGHT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
        }
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
        if(last_known_robot_side == RIGHT)
        {
          new_step_pos_temp[0] -= step_seperation[0];
          new_step_pos_temp[1] -= step_seperation[1];
          new_step_pos_temp[2] -= step_seperation[2];
        }
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepOffsetLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
      }
      else if(robot_side == RIGHT)
      {
        if(last_known_robot_side == LEFT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
        }
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
        if(last_known_robot_side == LEFT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
        }
      }
    }
  }
  return step_list_msg;
}

// Queue generators

ihmc_msgs::FootstepDataListRosMessage
Gait_Generation::createLowerBodyLinearGaitOffsetQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, x_vector destination, float heading)
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  // Retain previous ending point which would be the starting point
  int last_known_msg_id = step_list_msg.footstep_data_list.size() - 1;
  bool last_known_robot_side = step_list_msg.footstep_data_list[last_known_msg_id].robot_side;
  float last_step_offset;
  std::vector<float> last_known_pos(3, 0);
  std::vector<float> scnd_last_known_pos(3, 0);
  std::vector<float> starting_pos_offset(3, 0);
  last_known_pos[0] = step_list_msg.footstep_data_list[last_known_msg_id].location.x;
  last_known_pos[1] = step_list_msg.footstep_data_list[last_known_msg_id].location.y;
  last_known_pos[2] = step_list_msg.footstep_data_list[last_known_msg_id].location.z;
  ROS_INFO("LASTXYZ:\t%f\t%f\t%f", last_known_pos[0], last_known_pos[1], last_known_pos[2]);
  scnd_last_known_pos[0] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.x;
  scnd_last_known_pos[1] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.y;
  scnd_last_known_pos[2] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.z;
  starting_pos_offset[0] = last_known_pos[0] - scnd_last_known_pos[0];
  starting_pos_offset[1] = last_known_pos[1] - scnd_last_known_pos[1];
  starting_pos_offset[2] = last_known_pos[2] - scnd_last_known_pos[2];
  
  // Set constants for controlling linear gait
  const float combined_step_distance = DEFAULT_STEP_DISTANCE;
  const float foot_seperation = DEFAULT_STEP_SEPERATION;
  const float step_height = DEFAULT_STEP_HEIGHT;
  const float conversion_factor = combined_step_distance/sqrt(pow(destination.x, 2.0) + pow(destination.y, 2.0));
  const float step_distance_x = destination.x * conversion_factor;
  const float step_distance_y = destination.y * conversion_factor;
  const float step_distance_z = destination.z * conversion_factor;
  bool robot_side = last_known_robot_side;
  std::vector<float> new_step_pos(3, 0);
  std::vector<float> new_step_pos_temp(3, 0);
  
  std::vector<float> last_heading_temp(4,0);
  last_heading_temp[0] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.x;
  last_heading_temp[1] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.y;
  last_heading_temp[2] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.z;
  last_heading_temp[3] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.w;
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Quaternion last_known_heading(last_heading_temp[0],last_heading_temp[1],last_heading_temp[2],last_heading_temp[3]);
  tf2::Matrix3x3 last_heading(last_known_heading);
  last_heading.getRPY(roll, pitch, yaw);
  std::vector<float> new_step_seperation(3, 0);
  std::vector<float> step_seperation(3, 0);
  // Utilize previous msg to complete necessary information
  if(last_known_robot_side == LEFT)
  {
    yaw -= 0.06;
    new_step_seperation[1] = -DEFAULT_STEP_SEPERATION;
  }
  else if(last_known_robot_side == RIGHT)
  {
    yaw += 0.06;
    new_step_seperation[1] = DEFAULT_STEP_SEPERATION;
  }
  last_heading.setRPY(0.0, 0.0, yaw);
  step_seperation[0] += new_step_seperation[0]*last_heading[0][0] + new_step_seperation[1]*last_heading[0][1] + new_step_seperation[2]*last_heading[0][2];
  step_seperation[1] += new_step_seperation[0]*last_heading[1][0] + new_step_seperation[1]*last_heading[1][1] + new_step_seperation[2]*last_heading[1][2];
  step_seperation[2] += new_step_seperation[0]*last_heading[2][0] + new_step_seperation[1]*last_heading[2][1] + new_step_seperation[2]*last_heading[2][2];
  tf2::Matrix3x3 inverse_heading(last_heading.inverse());
  last_step_offset += starting_pos_offset[0]*inverse_heading[0][0] + starting_pos_offset[1]*inverse_heading[0][1] + starting_pos_offset[2]*inverse_heading[0][2];
  
  // Set destination with one step forward because of queueing nature
  // Also add step height offset for walking stabalization
  new_step_pos[0] = -last_step_offset;
  
  step_list_msg.footstep_data_list.pop_back();
  
  new_step_pos[0] = 0.0;
  new_step_pos[1] = 0.0;
  new_step_pos[2] = 0.0;
  ROS_INFO("Setpoint XYZ:\t%f\t%f\t%f", destination.x, destination.y, destination.z);
  if(fabs(destination.x)/4.0 >= fabs(destination.y))
  {
    while(fabs(new_step_pos[0]) <= fabs(destination.x) - 0.01)
    {
      if(fabs(destination.x) - fabs(new_step_pos[0]) < fabs(step_distance_x))
      {
        new_step_pos[0] = destination.x;
        new_step_pos[1] = destination.y;
        new_step_pos[2] = destination.z;
      }
      else
      {
        new_step_pos[0] += step_distance_x;
        new_step_pos[1] += step_distance_y;
        new_step_pos[2] += step_distance_z;
      }
      new_step_pos[2] = DEFAULT_STEP_HEIGHT;
      tf2::Matrix3x3 rotation;
      rotation.setRPY(0.0, 0.0, heading + yaw);
      new_step_pos_temp[0] = new_step_pos[0]*rotation[0][0] + new_step_pos[1]*rotation[0][1] + new_step_pos[2]*rotation[0][2];
      new_step_pos_temp[1] = new_step_pos[0]*rotation[1][0] + new_step_pos[1]*rotation[1][1] + new_step_pos[2]*rotation[1][2];
      new_step_pos_temp[2] = new_step_pos[0]*rotation[2][0] + new_step_pos[1]*rotation[2][1] + new_step_pos[2]*rotation[2][2];
      new_step_pos_temp[0] += last_known_pos[0];
      new_step_pos_temp[1] += last_known_pos[1];
      new_step_pos_temp[2] += last_known_pos[2];
      
      if(robot_side == LEFT)
      {
        if(last_known_robot_side == RIGHT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
        }
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
        robot_side = RIGHT;
      }
      else if(robot_side == RIGHT)
      {
        if(last_known_robot_side == LEFT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
        }
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
        robot_side = LEFT;
      }
    }
  }
  else
  {
    if(destination.y > 0)
    {
      robot_side = LEFT;
    }
    else
    {
      robot_side = RIGHT;
    }
    while(fabs(new_step_pos[1]) <= fabs(destination.y) - 0.01)
    {
      ROS_INFO("Creating side step");
      if(fabs(destination.y) - fabs(new_step_pos[1]) < fabs(step_distance_y)/4)
      {
        new_step_pos[0] = destination.x;
        new_step_pos[1] = destination.y;
        new_step_pos[2] = destination.z;
      }
      else
      {
        new_step_pos[0] += step_distance_x/4;
        new_step_pos[1] += step_distance_y/4;
        new_step_pos[2] += step_distance_z/4;
      }
      new_step_pos[2] = DEFAULT_STEP_HEIGHT;
      tf2::Matrix3x3 rotation;
      rotation.setRPY(0.0, 0.0, heading + yaw);
      new_step_pos_temp[0] = new_step_pos[0]*rotation[0][0] + new_step_pos[1]*rotation[0][1] + new_step_pos[2]*rotation[0][2];
      new_step_pos_temp[1] = new_step_pos[0]*rotation[1][0] + new_step_pos[1]*rotation[1][1] + new_step_pos[2]*rotation[1][2];
      new_step_pos_temp[2] = new_step_pos[0]*rotation[2][0] + new_step_pos[1]*rotation[2][1] + new_step_pos[2]*rotation[2][2];
      new_step_pos_temp[0] += last_known_pos[0];
      new_step_pos_temp[1] += last_known_pos[1];
      new_step_pos_temp[2] += last_known_pos[2];
      
      if(robot_side == LEFT)
      {
        ROS_INFO("Creating left side step");
        if(last_known_robot_side == RIGHT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack left: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
          new_step_pos_temp[0] -= step_seperation[0];
          new_step_pos_temp[1] -= step_seperation[1];
          new_step_pos_temp[2] -= step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack right: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
        }
        else
        {
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack left: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack right: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
        }
      }
      else if(robot_side == RIGHT)
      {
        ROS_INFO("Creating right side step");
        if(last_known_robot_side == LEFT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack right: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
          new_step_pos_temp[0] -= step_seperation[0];
          new_step_pos_temp[1] -= step_seperation[1];
          new_step_pos_temp[2] -= step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack left: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
        }
        else
        {
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack right: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack left: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
        }
      }
    }
  }
  return step_list_msg;
}

ihmc_msgs::FootstepDataListRosMessage
Gait_Generation::createLowerBodyLinearGaitOffsetQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, x_vector destination, float heading, float step_distance)
{
  step_list_msg.default_swing_time = DEFAULT_SWING_TIME*2;
  step_list_msg.default_transfer_time = DEFAULT_TRANSFER_TIME*2;
  step_list_msg.final_transfer_time = FINAL_TRANSFER_TIME*2;
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  // Retain previous ending point which would be the starting point
  int last_known_msg_id = step_list_msg.footstep_data_list.size() - 1;
  bool last_known_robot_side = step_list_msg.footstep_data_list[last_known_msg_id].robot_side;
  float last_step_offset;
  std::vector<float> last_known_pos(3, 0);
  std::vector<float> scnd_last_known_pos(3, 0);
  std::vector<float> starting_pos_offset(3, 0);
  last_known_pos[0] = step_list_msg.footstep_data_list[last_known_msg_id].location.x;
  last_known_pos[1] = step_list_msg.footstep_data_list[last_known_msg_id].location.y;
  last_known_pos[2] = step_list_msg.footstep_data_list[last_known_msg_id].location.z;
  ROS_INFO("LASTXYZ:\t%f\t%f\t%f", last_known_pos[0], last_known_pos[1], last_known_pos[2]);
  scnd_last_known_pos[0] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.x;
  scnd_last_known_pos[1] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.y;
  scnd_last_known_pos[2] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.z;
  starting_pos_offset[0] = last_known_pos[0] - scnd_last_known_pos[0];
  starting_pos_offset[1] = last_known_pos[1] - scnd_last_known_pos[1];
  starting_pos_offset[2] = last_known_pos[2] - scnd_last_known_pos[2];
  
  // Set constants for controlling linear gait
  const float combined_step_distance = step_distance;
  const float foot_seperation = DEFAULT_STEP_SEPERATION;
  const float step_height = DEFAULT_STEP_HEIGHT;
  const float conversion_factor = combined_step_distance/sqrt(pow(destination.x, 2.0) + pow(destination.y, 2.0));
  const float step_distance_x = destination.x * conversion_factor;
  const float step_distance_y = destination.y * conversion_factor;
  const float step_distance_z = destination.z * conversion_factor;
  bool robot_side = last_known_robot_side;
  std::vector<float> new_step_pos(3, 0);
  std::vector<float> new_step_pos_temp(3, 0);
  
  std::vector<float> last_heading_temp(4,0);
  last_heading_temp[0] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.x;
  last_heading_temp[1] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.y;
  last_heading_temp[2] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.z;
  last_heading_temp[3] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.w;
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Quaternion last_known_heading(last_heading_temp[0],last_heading_temp[1],last_heading_temp[2],last_heading_temp[3]);
  tf2::Matrix3x3 last_heading(last_known_heading);
  last_heading.getRPY(roll, pitch, yaw);
  std::vector<float> new_step_seperation(3, 0);
  std::vector<float> step_seperation(3, 0);
  // Utilize previous msg to complete necessary information
  if(last_known_robot_side == LEFT)
  {
    yaw -= 0.06;
    new_step_seperation[1] = -DEFAULT_STEP_SEPERATION;
  }
  else if(last_known_robot_side == RIGHT)
  {
    yaw += 0.06;
    new_step_seperation[1] = DEFAULT_STEP_SEPERATION;
  }
  last_heading.setRPY(0.0, 0.0, yaw);
  step_seperation[0] += new_step_seperation[0]*last_heading[0][0] + new_step_seperation[1]*last_heading[0][1] + new_step_seperation[2]*last_heading[0][2];
  step_seperation[1] += new_step_seperation[0]*last_heading[1][0] + new_step_seperation[1]*last_heading[1][1] + new_step_seperation[2]*last_heading[1][2];
  step_seperation[2] += new_step_seperation[0]*last_heading[2][0] + new_step_seperation[1]*last_heading[2][1] + new_step_seperation[2]*last_heading[2][2];
  tf2::Matrix3x3 inverse_heading(last_heading.inverse());
  last_step_offset += starting_pos_offset[0]*inverse_heading[0][0] + starting_pos_offset[1]*inverse_heading[0][1] + starting_pos_offset[2]*inverse_heading[0][2];
  
  // Set destination with one step forward because of queueing nature
  // Also add step height offset for walking stabalization
  new_step_pos[0] = -last_step_offset;
  
  step_list_msg.footstep_data_list.pop_back();
  
  new_step_pos[0] = 0.0;
  new_step_pos[1] = 0.0;
  new_step_pos[2] = DEFAULT_STEP_HEIGHT;
  ROS_INFO("Setpoint XYZ:\t%f\t%f\t%f", destination.x, destination.y, destination.z);
  if(fabs(destination.x)/4.0 >= fabs(destination.y))
  {
    while(fabs(new_step_pos[0]) <= fabs(destination.x) - 0.01)
    {
      if(fabs(destination.x) - fabs(new_step_pos[0]) < fabs(step_distance_x))
      {
        new_step_pos[0] = destination.x;
        new_step_pos[1] = destination.y;
        new_step_pos[2] = destination.z;
      }
      else
      {
        new_step_pos[0] += step_distance_x;
        new_step_pos[1] += step_distance_y;
        new_step_pos[2] += step_distance_z;
      }
      tf2::Matrix3x3 rotation;
      rotation.setRPY(0.0, 0.0, heading + yaw);
      new_step_pos_temp[0] = new_step_pos[0]*rotation[0][0] + new_step_pos[1]*rotation[0][1] + new_step_pos[2]*rotation[0][2];
      new_step_pos_temp[1] = new_step_pos[0]*rotation[1][0] + new_step_pos[1]*rotation[1][1] + new_step_pos[2]*rotation[1][2];
      new_step_pos_temp[2] = new_step_pos[0]*rotation[2][0] + new_step_pos[1]*rotation[2][1] + new_step_pos[2]*rotation[2][2];
      new_step_pos_temp[0] += last_known_pos[0];
      new_step_pos_temp[1] += last_known_pos[1];
      
      ROS_INFO("Step Height: %f", new_step_pos_temp[2]);
      if(robot_side == LEFT)
      {
        if(last_known_robot_side == RIGHT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
        }
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinearStairs(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
        robot_side = RIGHT;
      }
      else if(robot_side == RIGHT)
      {
        if(last_known_robot_side == LEFT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
        }
        step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinearStairs(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
        robot_side = LEFT;
      }
    }
  }
  else
  {
    if(destination.y > 0)
    {
      robot_side = LEFT;
    }
    else
    {
      robot_side = RIGHT;
    }
    while(fabs(new_step_pos[1]) <= fabs(destination.y) - 0.01)
    {
      ROS_INFO("Creating side step");
      if(fabs(destination.y) - fabs(new_step_pos[1]) < fabs(step_distance_y)/4)
      {
        new_step_pos[0] = destination.x;
        new_step_pos[1] = destination.y;
        new_step_pos[2] = destination.z;
      }
      else
      {
        new_step_pos[0] += step_distance_x/4;
        new_step_pos[1] += step_distance_y/4;
        new_step_pos[2] += step_distance_z/4;
      }
      new_step_pos[2] = DEFAULT_STEP_HEIGHT;
      tf2::Matrix3x3 rotation;
      rotation.setRPY(0.0, 0.0, heading + yaw);
      new_step_pos_temp[0] = new_step_pos[0]*rotation[0][0] + new_step_pos[1]*rotation[0][1] + new_step_pos[2]*rotation[0][2];
      new_step_pos_temp[1] = new_step_pos[0]*rotation[1][0] + new_step_pos[1]*rotation[1][1] + new_step_pos[2]*rotation[1][2];
      new_step_pos_temp[2] = new_step_pos[0]*rotation[2][0] + new_step_pos[1]*rotation[2][1] + new_step_pos[2]*rotation[2][2];
      new_step_pos_temp[0] += last_known_pos[0];
      new_step_pos_temp[1] += last_known_pos[1];
      new_step_pos_temp[2] += last_known_pos[2];
      
      if(robot_side == LEFT)
      {
        ROS_INFO("Creating left side step");
        if(last_known_robot_side == RIGHT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinearStairs(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack left: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
          new_step_pos_temp[0] -= step_seperation[0];
          new_step_pos_temp[1] -= step_seperation[1];
          new_step_pos_temp[2] -= step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinearStairs(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack right: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
        }
        else
        {
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinearStairs(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack left: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinearStairs(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack right: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
        }
      }
      else if(robot_side == RIGHT)
      {
        ROS_INFO("Creating right side step");
        if(last_known_robot_side == LEFT)
        {
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinearStairs(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack right: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
          new_step_pos_temp[0] -= step_seperation[0];
          new_step_pos_temp[1] -= step_seperation[1];
          new_step_pos_temp[2] -= step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinearStairs(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack left: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
        }
        else
        {
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(RIGHT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack right: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
          new_step_pos_temp[0] += step_seperation[0];
          new_step_pos_temp[1] += step_seperation[1];
          new_step_pos_temp[2] += step_seperation[2];
          step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, step_distance_x, new_step_pos_temp, heading + yaw));
          ROS_INFO("PushBack left: %f\t%f", new_step_pos_temp[0], new_step_pos_temp[1]);
        }
      }
    }
  }
  return step_list_msg;
}

ihmc_msgs::FootstepDataListRosMessage
Gait_Generation::createLowerBodyArcGaitQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, bool turn, float arc_length, float radian)
{
  // Retain previous ending point which would be the starting point
  int last_known_msg_id = step_list_msg.footstep_data_list.size() - 1;
  bool last_known_robot_side = step_list_msg.footstep_data_list[last_known_msg_id].robot_side;
  float last_step_offset;
  std::vector<float> last_known_pos(3, 0);
  std::vector<float> scnd_last_known_pos(3, 0);
  std::vector<float> starting_pos_offset(3, 0);
  last_known_pos[0] = step_list_msg.footstep_data_list[last_known_msg_id].location.x;
  last_known_pos[1] = step_list_msg.footstep_data_list[last_known_msg_id].location.y;
  last_known_pos[2] = step_list_msg.footstep_data_list[last_known_msg_id].location.z;
  scnd_last_known_pos[0] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.x;
  scnd_last_known_pos[1] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.y;
  scnd_last_known_pos[2] = step_list_msg.footstep_data_list[last_known_msg_id - 1].location.z;
  starting_pos_offset[0] = last_known_pos[0] - scnd_last_known_pos[0];
  starting_pos_offset[1] = last_known_pos[1] - scnd_last_known_pos[1];
  starting_pos_offset[2] = last_known_pos[2] - scnd_last_known_pos[2];
  
  std::vector<float> last_heading_temp(4,0);
  last_heading_temp[0] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.x;
  last_heading_temp[1] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.y;
  last_heading_temp[2] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.z;
  last_heading_temp[3] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.w;
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Quaternion last_known_heading(last_heading_temp[0],last_heading_temp[1],last_heading_temp[2],last_heading_temp[3]);
  tf2::Matrix3x3 last_heading(last_known_heading);
  last_heading.getRPY(roll, pitch, yaw);
  std::vector<float> new_step_seperation(3, 0);
  std::vector<float> step_seperation(3, 0);
  bool robot_side = last_known_robot_side;
  
  // Utilize previous msg to complete necessary information
  if(last_known_robot_side == LEFT)
  {
    yaw -= 0.06;
    new_step_seperation[1] = -DEFAULT_STEP_SEPERATION;
  }
  else if(last_known_robot_side == RIGHT)
  {
    yaw += 0.06;
    new_step_seperation[1] = DEFAULT_STEP_SEPERATION;
  }
  
  last_heading.setRPY(0.0, 0.0, yaw);
  
  step_seperation[0] += new_step_seperation[0]*last_heading[0][0] + new_step_seperation[1]*last_heading[0][1] + new_step_seperation[2]*last_heading[0][2];
  step_seperation[1] += new_step_seperation[0]*last_heading[1][0] + new_step_seperation[1]*last_heading[1][1] + new_step_seperation[2]*last_heading[1][2];
  step_seperation[2] += new_step_seperation[0]*last_heading[2][0] + new_step_seperation[1]*last_heading[2][1] + new_step_seperation[2]*last_heading[2][2];
  tf2::Matrix3x3 inverse_heading(last_heading.inverse());
  last_step_offset += starting_pos_offset[0]*inverse_heading[0][0] + starting_pos_offset[1]*inverse_heading[0][1] + starting_pos_offset[2]*inverse_heading[0][2];
  
  float step_distance = DEFAULT_STEP_DISTANCE;
  float foot_seperation = DEFAULT_STEP_SEPERATION;
  std::vector<float> new_step_pos(3, 0);
  std::vector<float> new_step_pos_temp(3, 0);
  
  // Pre calculates math for arc paths
  float num_steps = arc_length/step_distance;
  if(step_distance > arc_length)
  {
    num_steps = 4.0;
  }
  float radian_increment = radian/num_steps;
  foot_seperation += (0.2 * fabs(radian_increment)/M_PI);
  float radius = arc_length/radian;
  float radius_left = radius - foot_seperation/2.0;
  float radius_right = radius + foot_seperation/2.0;
  float current_radian_increment = 0.0;
  float foot_orient = current_radian_increment;
  
  // Switches direction of arc
  if(turn == RIGHT)
  {
    radius_left = radius + foot_seperation/2.0;
    radius_right = radius - foot_seperation/2.0;
  }
  
  current_radian_increment = -(last_step_offset/0.46 * radian_increment);
  step_list_msg.footstep_data_list.pop_back();
  
  while(current_radian_increment < radian)
  {
    new_step_pos[2] = DEFAULT_STEP_HEIGHT;
    if(current_radian_increment > radian - radian_increment)
    {
      current_radian_increment = radian;
    }
    else
    {
      current_radian_increment += radian_increment;
    }
    if(robot_side == LEFT)
    {
      new_step_pos[0] = radius_left * sin(current_radian_increment);
      if((current_radian_increment > M_PI/2 && current_radian_increment < M_PI*3.0/2.0) || (current_radian_increment < -M_PI/2 && current_radian_increment > -M_PI* 3.0/2.0))
      {
        new_step_pos[1] = -(-sqrt(pow(radius_left, 2.0) - pow(new_step_pos[0], 2.0)) - radius_left);
      }
      else
      {
        new_step_pos[1] = -(sqrt(pow(radius_left, 2.0) - pow(new_step_pos[0], 2.0)) - radius_left);
      }
      foot_orient = current_radian_increment;
      
      if(turn == RIGHT)
      {
        new_step_pos[1] = -new_step_pos[1];
        foot_orient = -foot_orient;
      }
      
      new_step_pos_temp[0] = new_step_pos[0]*last_heading[0][0] + new_step_pos[1]*last_heading[0][1] + new_step_pos[2]*last_heading[0][2];
      new_step_pos_temp[1] = new_step_pos[0]*last_heading[1][0] + new_step_pos[1]*last_heading[1][1] + new_step_pos[2]*last_heading[1][2];
      new_step_pos_temp[2] = new_step_pos[0]*last_heading[2][0] + new_step_pos[1]*last_heading[2][1] + new_step_pos[2]*last_heading[2][2];
      new_step_pos[0] = new_step_pos_temp[0] + last_known_pos[0];
      new_step_pos[1] = new_step_pos_temp[1] + last_known_pos[1];
      new_step_pos[2] = new_step_pos_temp[2] + last_known_pos[2];
      
      if(last_known_robot_side == RIGHT)
      {
        new_step_pos[0] += step_seperation[0];
        new_step_pos[1] += step_seperation[1];
        new_step_pos[2] += step_seperation[2];
      }
       
      step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepArc(LEFT, step_distance, new_step_pos, foot_orient + yaw));
      robot_side = RIGHT;
    }
    else
    {
      new_step_pos[0] = radius_right * sin(current_radian_increment);
      if((current_radian_increment > M_PI/2 && current_radian_increment < M_PI*3.0/2.0) || (current_radian_increment < -M_PI/2 && current_radian_increment > -M_PI* 3.0/2.0))
      {
        new_step_pos[1] = -(-sqrt(pow(radius_right, 2.0) - pow(new_step_pos[0], 2.0)) - radius_right);
      }
      else
      {
        new_step_pos[1] = -(sqrt(pow(radius_right, 2.0) - pow(new_step_pos[0], 2.0)) - radius_right);
      }
      foot_orient = current_radian_increment;
      
      if(turn == RIGHT)
      {
        new_step_pos[1] = -new_step_pos[1];
        foot_orient = -foot_orient;
      }
      
      new_step_pos_temp[0] = new_step_pos[0]*last_heading[0][0] + new_step_pos[1]*last_heading[0][1] + new_step_pos[2]*last_heading[0][2];
      new_step_pos_temp[1] = new_step_pos[0]*last_heading[1][0] + new_step_pos[1]*last_heading[1][1] + new_step_pos[2]*last_heading[1][2];
      new_step_pos_temp[2] = new_step_pos[0]*last_heading[2][0] + new_step_pos[1]*last_heading[2][1] + new_step_pos[2]*last_heading[2][2];
      new_step_pos[0] = new_step_pos_temp[0] + last_known_pos[0];
      new_step_pos[1] = new_step_pos_temp[1] + last_known_pos[1];
      new_step_pos[2] = new_step_pos_temp[2] + last_known_pos[2];
      
      if(last_known_robot_side == LEFT)
      {
        new_step_pos[0] += step_seperation[0];
        new_step_pos[1] += step_seperation[1];
        new_step_pos[2] += step_seperation[2];
      }
      
      step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepArc(RIGHT, step_distance, new_step_pos, foot_orient + yaw));
      robot_side = LEFT;
    }
  }
  return step_list_msg;
}

ihmc_msgs::FootstepDataListRosMessage
Gait_Generation::createLowerBodyEndStepQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg)
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  // Retain previous ending point which would be the starting point
  int last_known_msg_id = step_list_msg.footstep_data_list.size() - 1;
  bool last_known_robot_side = step_list_msg.footstep_data_list[last_known_msg_id].robot_side;
  std::vector<float> new_step_pos(3, 0);
  new_step_pos[0] = step_list_msg.footstep_data_list[last_known_msg_id].location.x;
  new_step_pos[1] = step_list_msg.footstep_data_list[last_known_msg_id].location.y;
  new_step_pos[2] = step_list_msg.footstep_data_list[last_known_msg_id].location.z;
  
  std::vector<float> last_heading_temp(4,0);
  last_heading_temp[0] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.x;
  last_heading_temp[1] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.y;
  last_heading_temp[2] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.z;
  last_heading_temp[3] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.w;
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Quaternion last_known_heading(last_heading_temp[0],last_heading_temp[1],last_heading_temp[2],last_heading_temp[3]);
  tf2::Matrix3x3 last_heading(last_known_heading);
  last_heading.getRPY(roll, pitch, yaw);
  std::vector<float> new_step_seperation(3, 0);
  bool robot_side;
  // Utilize previous msg to complete necessary information
  if(last_known_robot_side == LEFT)
  {
    robot_side = RIGHT;
    yaw -= 0.06;
    new_step_seperation[1] = -DEFAULT_STEP_SEPERATION;
  }
  else if(last_known_robot_side == RIGHT)
  {
    robot_side = LEFT;
    yaw += 0.06;
    new_step_seperation[1] = DEFAULT_STEP_SEPERATION;
  }
  last_heading.setRPY(0.0, 0.0, yaw);
  
  new_step_pos[0] += new_step_seperation[0]*last_heading[0][0] + new_step_seperation[1]*last_heading[0][1] + new_step_seperation[2]*last_heading[0][2];
  new_step_pos[1] += new_step_seperation[0]*last_heading[1][0] + new_step_seperation[1]*last_heading[1][1] + new_step_seperation[2]*last_heading[1][2];
  new_step_pos[2] += new_step_seperation[0]*last_heading[2][0] + new_step_seperation[1]*last_heading[2][1] + new_step_seperation[2]*last_heading[2][2];
  if(robot_side == LEFT)
  {
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(LEFT, 0.0, new_step_pos, yaw + 0.06));
    robot_side = RIGHT;
  }
  else if(robot_side == RIGHT)
  {
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinear(RIGHT, 0.0, new_step_pos, yaw - 0.06));
    robot_side = LEFT;
  }
  return step_list_msg;
}

ihmc_msgs::FootstepDataListRosMessage
Gait_Generation::createLowerBodyEndStepStairQueue(ihmc_msgs::FootstepDataListRosMessage step_list_msg, float height)
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  // Retain previous ending point which would be the starting point
  int last_known_msg_id = step_list_msg.footstep_data_list.size() - 1;
  bool last_known_robot_side = step_list_msg.footstep_data_list[last_known_msg_id].robot_side;
  std::vector<float> new_step_pos(3, 0);
  new_step_pos[0] = step_list_msg.footstep_data_list[last_known_msg_id].location.x;
  new_step_pos[1] = step_list_msg.footstep_data_list[last_known_msg_id].location.y;
  new_step_pos[2] = step_list_msg.footstep_data_list[last_known_msg_id].location.z;
  
  std::vector<float> last_heading_temp(4,0);
  last_heading_temp[0] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.x;
  last_heading_temp[1] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.y;
  last_heading_temp[2] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.z;
  last_heading_temp[3] = step_list_msg.footstep_data_list[last_known_msg_id].orientation.w;
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Quaternion last_known_heading(last_heading_temp[0],last_heading_temp[1],last_heading_temp[2],last_heading_temp[3]);
  tf2::Matrix3x3 last_heading(last_known_heading);
  last_heading.getRPY(roll, pitch, yaw);
  std::vector<float> new_step_seperation(3, 0);
  bool robot_side;
  // Utilize previous msg to complete necessary information
  if(last_known_robot_side == LEFT)
  {
    robot_side = RIGHT;
    yaw -= 0.06;
    new_step_seperation[1] = -DEFAULT_STEP_SEPERATION;
  }
  else if(last_known_robot_side == RIGHT)
  {
    robot_side = LEFT;
    yaw += 0.06;
    new_step_seperation[1] = DEFAULT_STEP_SEPERATION;
  }
  last_heading.setRPY(0.0, 0.0, yaw);
  
  new_step_pos[0] += new_step_seperation[0]*last_heading[0][0] + new_step_seperation[1]*last_heading[0][1] + new_step_seperation[2]*last_heading[0][2];
  new_step_pos[1] += new_step_seperation[0]*last_heading[1][0] + new_step_seperation[1]*last_heading[1][1] + new_step_seperation[2]*last_heading[1][2];
  new_step_pos[2] = height;
  if(robot_side == LEFT)
  {
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinearStairs(LEFT, 0.0, new_step_pos, yaw + 0.06));
    robot_side = RIGHT;
  }
  else if(robot_side == RIGHT)
  {
    step_list_msg.footstep_data_list.push_back(this->trajectory_generation.createStepLinearStairs(RIGHT, 0.0, new_step_pos, yaw - 0.06));
    robot_side = LEFT;
  }
  return step_list_msg;
}
