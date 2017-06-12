#include "habitat_driver/habitat_driver.hpp"
#include "x_publisher/x_publisher.hpp"


Habitat_Driver::Habitat_Driver()
{}

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
  x_publisher.publish(gait_generation.createStanceReset().trajectory_list_msg);
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
  left_arm_msg.unique_id = 0;
  right_arm_msg.unique_id = 0;
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
  pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(0.5, 1.15));
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.65;
  new_chest_orient[2] = yaw;
  float chest_time = 6.0;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(chest_time, new_chest_orient, new_chest_ang_vel));
  
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
  destination.x = distance_to_stairs - 0.348;
  destination.y = 0;
  destination.z = 0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOffsetQueue(step_list_msg, destination, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  x_publisher.publish(step_list_msg);
  
  destination.x = 2.23;
  destination.y = 0;
  destination.z = 1.8;
  step_list_msg = gait_generation.createLowerBodyLinearGaitStairOverride(destination, 0.0, 0.25);
  step_list_msg = gait_generation.createLowerBodyEndStepStairQueue(step_list_msg, 1.8);
  x_publisher.publish(trajectory_list_msg);
  ros::Duration(1.0).sleep();
  x_publisher.publish(step_list_msg);
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
  end_point[0] = 3.525;
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
  step_list_msg = gait_generation.createLowerBodyEndStepStairQueue(step_list_msg, 1.8);
  x_publisher.publish(step_list_msg);
}

void
Habitat_Driver::openDoor()
{
  X_Publisher x_publisher;
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
  right_arm_msg.unique_id = 0;
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
      trajectory_out[0] = -1.1;
      trajectory_out[1] = -1.3;
      trajectory_out[2] = 0.5;
      trajectory_out[3] = -1.2;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = -0.60;
      trajectory_out[6] = -0.28;
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
      trajectory_out[0] = -0.5;
      trajectory_out[1] = -1.3;
      trajectory_out[2] = 0.3;
      trajectory_out[3] = -0.8;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = -0.60;
      trajectory_out[6] = -0.28;
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
      trajectory_out[3] = -1.1;
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
  
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  x_vector destination;
  destination.x = 2.25;
  destination.y = -0.15;
  destination.z = 0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOverride(destination, 0.0);
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
  destination.x = 2.5;
  destination.y = 0;
  destination.z = 0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOffsetQueue(step_list_msg, destination, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  x_publisher.publish(step_list_msg);
}
