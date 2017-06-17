#include "habitat_driver/habitat_driver.hpp"
#include "x_publisher/x_publisher.hpp"


Habitat_Driver::Habitat_Driver()
{
  trajectoryRightHandPublisher = habNodeHandle.advertise<std_msgs::Float64MultiArray>("/right_hand_position_controller/command", 1);
  trajectoryLeftHandPublisher = habNodeHandle.advertise<std_msgs::Float64MultiArray>("/left_hand_position_controller/command", 1);
}

void
Habitat_Driver::climbStairs(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer->lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
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
  if(left_yaw < -M_PI_2 && right_yaw > M_PI_2)
  {
    if(yaw > 0)
    {
      yaw -= M_PI;
    }
    else
    {
      yaw += M_PI;
    }
  }
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
  X_Publisher x_publisher;
  ros::Duration(2.0).sleep();
  
  ROS_INFO("heading: %f", yaw);
  
  int num_angle = round(yaw / M_PI_4);
  float heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", heading);
  
  std::vector<float> yellow_stair_point(3, 0);
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 255 && map->points[point_id].r == 255)
    {
      yellow_stair_point[0] = map->points[point_id].x;
      yellow_stair_point[1] = map->points[point_id].y;
      yellow_stair_point[2] = map->points[point_id].z;
    }
  }
  float distance_to_stairs = sqrt(pow(fabs(yellow_stair_point[0] - center_point.x), 2.0) + pow(fabs(yellow_stair_point[1] - center_point.y), 2.0));
  
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
  
  pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(1.5, 1.15));
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.65;
  new_chest_orient[2] = yaw;
  float chest_time = 6.0;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(chest_time, new_chest_orient, new_chest_ang_vel));
  
  std::vector<float> left_arm_swing = getArmAtStair(LEFT);
  std::vector<float> ang_vel(7, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(3.0, left_arm_swing, ang_vel, left_arm_msg);
  std::vector<float> right_arm_swing = getArmAtStair(RIGHT);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(3.0, right_arm_swing, ang_vel, right_arm_msg);
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  ros::Duration(1.0).sleep();
  
  ros::Duration(0.01).sleep();
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading - yaw);
  x_vector destination;
  destination.x = distance_to_stairs - 0.295;
  destination.y = 0;
  destination.z = 0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOffsetQueue(step_list_msg, destination, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  x_publisher.publish(step_list_msg);
  
  destination.x = 2.22;
  destination.y = 0;
  destination.z = 1.8;
  step_list_msg = gait_generation.createLowerBodyLinearGaitStairOverride(destination, 0.0, 0.244);
  step_list_msg = gait_generation.createLowerBodyEndStepStairQueue(step_list_msg, 1.8 - 0.16);
  x_publisher.publish(trajectory_list_msg);
  ros::Duration(1.0).sleep();
  x_publisher.publish(step_list_msg);
}

std::vector<float>
Habitat_Driver::getArmAtStair(bool robot_side)
{
  std::vector<float> arm_at_rest(7, 0);
  
  if(robot_side == LEFT)
  {
    arm_at_rest[0] = -0.4;
    arm_at_rest[1] = -1.33;
    arm_at_rest[2] = 0.6;
    arm_at_rest[3] = -1.7;
    arm_at_rest[4] = 1.2;
  }
  else if(robot_side == RIGHT)
  {
    arm_at_rest[0] = -0.4;
    arm_at_rest[1] = 1.33;
    arm_at_rest[2] = 0.6;
    arm_at_rest[3] = 1.7;
    arm_at_rest[4] = 1.2;
  }
  
  return arm_at_rest;
}

void
Habitat_Driver::walkToDoor(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer->lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
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
  if(left_yaw < -M_PI_2 && right_yaw > M_PI_2)
  {
    if(yaw > 0)
    {
      yaw -= M_PI;
    }
    else
    {
      yaw += M_PI;
    }
  }
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
  X_Publisher x_publisher;
  
  ROS_INFO("heading: %f", yaw);
  
  int num_angle = round(yaw / M_PI_4);
  float heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", heading);
  
  std::vector<float> yellow_stair_point(3, 0);
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 255 && map->points[point_id].r == 255)
    {
      yellow_stair_point[0] = map->points[point_id].x;
      yellow_stair_point[1] = map->points[point_id].y;
      yellow_stair_point[2] = map->points[point_id].z;
    }
  }
  
  std::vector<float> end_point(3, 0);
  end_point[0] = 3.4;
  end_point[1] = 0.0;
  end_point[2] = 0.0;
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, heading);
  
  std::vector<float> task_point(3, 0);
  task_point[0] = end_point[0]*rotation[0][0] + end_point[1]*rotation[0][1] + end_point[2]*rotation[0][2];
  task_point[1] = end_point[0]*rotation[1][0] + end_point[1]*rotation[1][1] + end_point[2]*rotation[1][2];
  task_point[2] = end_point[0]*rotation[2][0] + end_point[1]*rotation[2][1] + end_point[2]*rotation[2][2];
  
  x_publisher.publish(gait_generation.createStanceReset().trajectory_list_msg);
  ros::Duration(2.0).sleep();
  
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading - yaw);
  x_vector destination;
  destination.x = task_point[0] + yellow_stair_point[0];
  destination.y = task_point[1] + yellow_stair_point[1];
  destination.z = task_point[2] + yellow_stair_point[2];
  step_list_msg = gait_generation.createLowerBodyLinearGaitQueue(step_list_msg, destination, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  x_publisher.publish(step_list_msg);
  x_publisher.publish(gait_generation.createStanceReset().trajectory_list_msg);
}

void
Habitat_Driver::alignToDoor(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer->lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
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
  if(left_yaw < -M_PI_2 && right_yaw > M_PI_2)
  {
    if(yaw > 0)
    {
      yaw -= M_PI;
    }
    else
    {
      yaw += M_PI;
    }
  }
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
  X_Publisher x_publisher;
  
  ROS_INFO("heading: %f", yaw);
  
  int num_angle = round(yaw / M_PI_4);
  float heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", heading);
  
  std::vector<float> yellow_stair_point(3, 0);
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 10 && map->points[point_id].g == 10 && map->points[point_id].r == 10)
    {
      yellow_stair_point[0] = map->points[point_id].x;
      yellow_stair_point[1] = map->points[point_id].y;
      yellow_stair_point[2] = map->points[point_id].z;
    }
  }
  
  std::vector<float> end_point(3, 0);
  end_point[0] = -0.76;
  end_point[1] = 0.0;
  end_point[2] = 0.0;
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, heading);
  
  std::vector<float> task_point(3, 0);
  task_point[0] = end_point[0]*rotation[0][0] + end_point[1]*rotation[0][1] + end_point[2]*rotation[0][2];
  task_point[1] = end_point[0]*rotation[1][0] + end_point[1]*rotation[1][1] + end_point[2]*rotation[1][2];
  task_point[2] = end_point[0]*rotation[2][0] + end_point[1]*rotation[2][1] + end_point[2]*rotation[2][2];
  
  x_publisher.publish(gait_generation.createStanceReset().trajectory_list_msg);
  ros::Duration(2.0).sleep();
  
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading - yaw);
  x_vector destination;
  destination.x = task_point[0] + yellow_stair_point[0];
  destination.y = task_point[1] + yellow_stair_point[1];
  destination.z = task_point[2] + yellow_stair_point[2];
  step_list_msg = gait_generation.createLowerBodyLinearGaitQueue(step_list_msg, destination, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  x_publisher.publish(step_list_msg);
}

void
Habitat_Driver::openDoor()
{
  X_Publisher x_publisher;
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
  right_arm_msg.unique_id = 0;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 1;
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
  
  std::vector<float> left_arm_swing;
  std::vector<float> ang_vel(7, 0);
  int num_turns;
  left_arm_swing = openDoorTrajectory(LEFT, 2);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(0.5, left_arm_swing, ang_vel, left_arm_msg);
  for(num_turns = 1; num_turns <= 5; num_turns++)
  {
    left_arm_swing = openDoorTrajectory(LEFT, 0);
    left_arm_msg = trajectory_generation.appendTrajectoryPoint(3*num_turns - 2, left_arm_swing, ang_vel, left_arm_msg);
    left_arm_swing = openDoorTrajectory(LEFT, 1);
    left_arm_msg = trajectory_generation.appendTrajectoryPoint(3*num_turns - 1, left_arm_swing, ang_vel, left_arm_msg);
    left_arm_swing = openDoorTrajectory(LEFT, 2);
    left_arm_msg = trajectory_generation.appendTrajectoryPoint(3*num_turns, left_arm_swing, ang_vel, left_arm_msg);
  }
  
  left_arm_swing = openDoorTrajectory(LEFT, 3);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(3*num_turns + 1, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = openDoorTrajectory(LEFT, 4);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(3*num_turns + 2, left_arm_swing, ang_vel, left_arm_msg);
  
  int num_angle = round(yaw / M_PI_4);
  float chest_heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", chest_heading);
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.3;
  new_chest_orient[2] = chest_heading;
  float chest_time = 1.0;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(1.0, new_chest_orient, new_chest_ang_vel));
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  
  x_publisher.publish(trajectory_list_msg);
  ros::Duration(3*num_turns + 3).sleep();
}

std::vector<float>
Habitat_Driver::openDoorTrajectory(bool robot_side_arm, int step_num)
{
  std::vector<float> trajectory_out(7, 0);
  if(step_num == 0)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.57;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 0.8;
      trajectory_out[3] = -0.8;
      trajectory_out[4] = 1.4;
      trajectory_out[5] = -0.0;
      trajectory_out[6] = -0.0;
    }
    else
    {
      trajectory_out[0] = -1.1;
      trajectory_out[1] = 1.28;
      trajectory_out[2] = 0.5;
      trajectory_out[3] = 1.2;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.60;
      trajectory_out[6] = 0.28;
    }
  }
  if(step_num == 1)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.9;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 0.8;
      trajectory_out[3] = -0.4;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = -0.0;
      trajectory_out[6] = -0.0;
    }
    else
    {
      trajectory_out[0] = -0.5;
      trajectory_out[1] = 1.28;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 0.8;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.60;
      trajectory_out[6] = 0.28;
    }
  }
  if(step_num == 2)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.8;
      trajectory_out[1] = -1.0;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = -1.3;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = -0.0;
      trajectory_out[6] = -0.2;
    }
    else
    {
      trajectory_out[0] = -0.8;
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
      trajectory_out[0] = -0.8;
      trajectory_out[1] = -1.4;
      trajectory_out[2] = 0.8;
      trajectory_out[3] = -2.0;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = -0.0;
      trajectory_out[6] = -0.2;
    }
    else
    {
      trajectory_out[0] = -0.8;
      trajectory_out[1] = 1.4;
      trajectory_out[2] = 0.8;
      trajectory_out[3] = 2.0;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.2;
    }
  }
  if(step_num == 4)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.1;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = -1.0;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = -0.0;
      trajectory_out[6] = -0.2;
    }
    else
    {
      trajectory_out[0] = -1.1;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 1.0;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.2;
    }
  }
  return trajectory_out;
}

void
Habitat_Driver::enterHab()
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
  
  float heading = -M_PI/2;
  ROS_INFO("Heading: %f", yaw);
  int num_angle = round(yaw / M_PI_4);
  float alignment_heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", alignment_heading);
  
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(alignment_heading - yaw);
  x_vector destination;
  destination.x = 2.1;
  destination.y = -0.15;
  destination.z = 0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOffsetQueue(step_list_msg, destination, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  if(heading > 3*M_PI/4 || heading < -3*M_PI/4)
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/4);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/4);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/4);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/4);
  }
  else if(heading > M_PI/2 || heading < -M_PI/2)
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/3);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/3);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/3);
  }
  else if(heading > M_PI/4 || heading < -M_PI/4)
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/2);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/2);
  }
  else
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading);
  }
  destination.x = 1.8;
  destination.y = 0;
  destination.z = 0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOffsetQueue(step_list_msg, destination, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  x_publisher.publish(step_list_msg);
  x_publisher.publish(gait_generation.createStanceReset().trajectory_list_msg);
}
void
Habitat_Driver::alignToDetector(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer->lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
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
  if(left_yaw < -M_PI_2 && right_yaw > M_PI_2)
  {
    if(yaw > 0)
    {
      yaw -= M_PI;
    }
    else
    {
      yaw += M_PI;
    }
  }
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
  X_Publisher x_publisher;
  
  
  std::vector<float> detector_point(3, 0);
  
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 200 && input->points[point_id].b == 200 && input->points[point_id].r == 50)
    {
      detector_point[0] += input->points[point_id].x;
      detector_point[1] += input->points[point_id].y;
      detector_point[2] += input->points[point_id].z;
    }
  }
  
  float detector_distance = sqrt(pow(fabs(detector_point[0] - center_point.x), 2.0) + pow(fabs(detector_point[1] - center_point.y), 2.0));
  float heading = asin((detector_point[1] - center_point.y)/detector_distance);
  if(detector_point[0] - center_point.x <= 0)
  {
    if(detector_point[1] - center_point.y <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  
  std::vector<float> end_point(3, 0);
  end_point[0] = -0.73;
  end_point[1] = 0.0;
  end_point[2] = 0.0;
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, heading);
  
  std::vector<float> task_point(3, 0);
  task_point[0] = end_point[0]*rotation[0][0] + end_point[1]*rotation[0][1] + end_point[2]*rotation[0][2];
  task_point[1] = end_point[0]*rotation[1][0] + end_point[1]*rotation[1][1] + end_point[2]*rotation[1][2];
  task_point[2] = end_point[0]*rotation[2][0] + end_point[1]*rotation[2][1] + end_point[2]*rotation[2][2];
  
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading - yaw);
  x_vector destination;
  destination.x = task_point[0] + detector_point[0];
  destination.y = task_point[1] + detector_point[1];
  destination.z = task_point[2] + detector_point[2];
  step_list_msg = gait_generation.createLowerBodyLinearGaitQueue(step_list_msg, destination, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  x_publisher.publish(step_list_msg);
}

void
Habitat_Driver::pickUpDetector(pcl::PointCloud<pcl::PointXYZRGB>* input)
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
  if(left_yaw < -M_PI_2 && right_yaw > M_PI_2)
  {
    if(yaw > 0)
    {
      yaw -= M_PI;
    }
    else
    {
      yaw += M_PI;
    }
  }
  
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
  std::vector<float> detector_tip_point(3, 0);
  std::vector<float> detector_handle_point(3, 0);
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 10 && input->points[point_id].r == 10 && input->points[point_id].b == 10)
    {
      detector_handle_point[0] = input->points[point_id].x;
      detector_handle_point[1] = input->points[point_id].y;
      detector_handle_point[2] = input->points[point_id].z;
    }
    if(input->points[point_id].g == 200 && input->points[point_id].r == 50 && input->points[point_id].b == 200)
    {
      detector_tip_point[0] = input->points[point_id].x;
      detector_tip_point[1] = input->points[point_id].y;
      detector_tip_point[2] = input->points[point_id].z;
    }
  }
  
  float handle_tip_distance = sqrt(pow(fabs(detector_tip_point[0] - detector_handle_point[0]), 2.0) + pow(fabs(detector_tip_point[1] - detector_handle_point[1]), 2.0));
  float detector_orient = asin((detector_tip_point[1] - detector_handle_point[1])/handle_tip_distance);
  if(detector_tip_point[0] - detector_handle_point[0] <= 0)
  {
    if(detector_tip_point[1] - detector_handle_point[1] <= 0)
    {
      detector_orient = -M_PI-detector_orient;
    }
    else
    {
      detector_orient = M_PI-detector_orient;
    }
  }
  ROS_INFO("Detector orient: %f", detector_orient);
  
  float handle_final_distance = sqrt(pow(fabs((detector_tip_point[0] + detector_handle_point[0])/2 - center_point.x), 2.0) + pow(fabs((detector_tip_point[1] + detector_handle_point[1])/2 - center_point.y), 2.0));
  float heading = asin(((detector_tip_point[1] + detector_handle_point[1])/2 - center_point.y)/handle_final_distance);
  if((detector_tip_point[0] + detector_handle_point[0])/2 - center_point.x <= 0)
  {
    if((detector_tip_point[1] + detector_handle_point[1])/2 - center_point.y <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  
  float relative_detector_orient = detector_orient - heading;
  if(relative_detector_orient < -M_PI)
  {
    relative_detector_orient += 2*M_PI;
  }
  else if(relative_detector_orient > M_PI)
  {
    relative_detector_orient -= 2*M_PI;
  }
  
  ROS_INFO("Relative orient: %f", relative_detector_orient);
  
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
  
  std::vector<float> left_arm_swing = grabComponentTrajectory(LEFT, 0);
  std::vector<float> ang_vel(7, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(6.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = grabComponentTrajectory(LEFT, 1);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(9.0, left_arm_swing, ang_vel, left_arm_msg);
  
  std::vector<float> right_arm_swing = grabComponentTrajectory(RIGHT, 0);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabComponentTrajectory(RIGHT, 1);
  right_arm_swing[5] -= relative_detector_orient/8;
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(4.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabComponentTrajectory(RIGHT, 2);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(7.0, right_arm_swing, ang_vel, right_arm_msg);
  
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.6;
  new_chest_orient[2] = yaw + 0.4;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(1.0, new_chest_orient, new_chest_ang_vel));
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(4.0, new_chest_orient, new_chest_ang_vel));
  new_chest_orient[1] = 0.2;
  new_chest_orient[2] = yaw + 0.0;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(7.0, new_chest_orient, new_chest_ang_vel));
  //
  //pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  //pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(0.5, 0.95));
  
  //Publish All Trajectories
  trajectory_list_msg.left_arm_trajectory_message = left_arm_msg;
  trajectory_list_msg.right_arm_trajectory_message = right_arm_msg;
  trajectory_list_msg.left_hand_trajectory_message = left_hand_msg;
  trajectory_list_msg.right_hand_trajectory_message = right_hand_msg;
  trajectory_list_msg.chest_trajectory_message = chest_msg;
  trajectory_list_msg.pelvis_trajectory_message = pelvis_msg;
  trajectory_list_msg.left_foot_trajectory_message = left_foot_msg;
  trajectory_list_msg.right_foot_trajectory_message = right_foot_msg;
  x_publisher.publish(trajectory_list_msg);
  std_msgs::Float64MultiArray left_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 1.5;
  finger_pos[1] = -0.0;
  finger_pos[2] = -0.0;
  finger_pos[3] = -0.0;
  finger_pos[4] = -0.0;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
  std_msgs::Float64MultiArray right_fingers;
  finger_pos[0] = 1.3;
  finger_pos[1] = 0.0;
  finger_pos[2] = 0.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(4.0).sleep();
  finger_pos[0] = 1.5;
  finger_pos[1] = 0.8;
  finger_pos[2] = 0.8;
  finger_pos[3] = 0.9;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  //ros::Duration(3.0).sleep();
  //finger_pos[0] = 1.5;
  //finger_pos[1] = 0.8;
  //finger_pos[2] = 0.8;
  //finger_pos[3] = 0.8;
  //finger_pos[4] = 0.8;
  //right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  //trajectoryRightHandPublisher.publish(right_fingers);
}

std::vector<float>
Habitat_Driver::grabComponentTrajectory(bool robot_side_arm, int step_num)
{
  std::vector<float> trajectory_out(7, 0);
  if(step_num == 0)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.5;
      trajectory_out[1] = -1.3;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -1.4;
      trajectory_out[4] = 0.8;
      trajectory_out[5] = 0.4;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.8;
      trajectory_out[1] = -0.2;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = 1.8;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
  }
  if(step_num == 1)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.8;
      trajectory_out[1] = -1.3;
      trajectory_out[2] = 0.6;
      trajectory_out[3] = -1.6;
      trajectory_out[4] = 0.6;
      trajectory_out[5] = 0.4;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.46;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 1.3;
      trajectory_out[3] = 0.3;
      trajectory_out[4] = 1.7;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
  }
  if(step_num == 2)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -1.8;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.46;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 1.3;
      trajectory_out[3] = 1.2;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
  }
  if(step_num == 3)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -1.8;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -0.7;
      trajectory_out[1] = 0.8;
      trajectory_out[2] = 1.67;
      trajectory_out[3] = 0.3;
      trajectory_out[4] = 1.3;
      trajectory_out[5] = -0.6;
      trajectory_out[6] = 0.2;
    }
  }
  if(step_num == 4)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -1.8;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -0.75;
      trajectory_out[1] = 0.5;
      trajectory_out[2] = 1.7;
      trajectory_out[3] = 2.1;
      trajectory_out[4] = 0.5;
      trajectory_out[5] = -0.6;
      trajectory_out[6] = -0.1;
    }
  }
  if(step_num == 5)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = -0.4;
      trajectory_out[3] = -1.8;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.7;
      trajectory_out[1] = 1.3;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = 0.6;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 6)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -1.8;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = 1.0;
      trajectory_out[1] = 1.3;
      trajectory_out[2] = -1.5;
      trajectory_out[3] = 1.6;
      trajectory_out[4] = 2.8;
      trajectory_out[5] = 0.6;
      trajectory_out[6] = -0.0;
    }
  }
  if(step_num == 7)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -2.0;
      trajectory_out[1] = -0.3;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = -0.3;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = 0.9;
      trajectory_out[1] = 1.3;
      trajectory_out[2] = -1.5;
      trajectory_out[3] = 1.6;
      trajectory_out[4] = 1.6;
      trajectory_out[5] = 0.6;
      trajectory_out[6] = -0.0;
    }
  }
  if(step_num == 8)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.9;
      trajectory_out[1] = -1.3;
      trajectory_out[2] = -0.6;
      trajectory_out[3] = -0.2;
      trajectory_out[4] = 3.1;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.57;
      trajectory_out[1] = -0.4;
      trajectory_out[2] = 0.6;
      trajectory_out[3] = 2.0;
      trajectory_out[4] = -1.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
  }
  return trajectory_out;
}
