#include "solar_components_driver/solar_components_driver.hpp"
#include "ihmc_msgs/ArmTrajectoryRosMessage.h"
#include "x_publisher/x_publisher.hpp"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "srcsim/Satellite.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"

Solar_Components_Driver::Solar_Components_Driver()
{
  stepListPublisher = dishNodeHandle.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list", 1);
  trajectoryListPublisher = dishNodeHandle.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/whole_body_trajectory", 1);
  trajectoryNeckPublisher = dishNodeHandle.advertise<ihmc_msgs::NeckTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/neck_trajectory", 1);
  trajectoryRightHandPublisher = dishNodeHandle.advertise<std_msgs::Float64MultiArray>("/right_hand_position_controller/command", 1);
  trajectoryLeftHandPublisher = dishNodeHandle.advertise<std_msgs::Float64MultiArray>("/left_hand_position_controller/command", 1);
}

void
Solar_Components_Driver::alignToPanel(Map_Scan* map_scan, tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  X_Publisher x_publisher;
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
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
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
  
  std::vector<float> button_point(3, 0);
  std::vector<float> handle_point(3, 0);
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
    {
      button_point[0] = map->points[point_id].x;
      button_point[1] = map->points[point_id].y;
      button_point[2] = map->points[point_id].z;
    }
    else if(map->points[point_id].g == 0 && map->points[point_id].r == 0 && map->points[point_id].b == 200)
    {
      handle_point[0] = map->points[point_id].x;
      handle_point[1] = map->points[point_id].y;
      handle_point[2] = map->points[point_id].z;
    }
  }
  
  float handle_final_distance = sqrt(pow(fabs(handle_point[0] - center_point.x), 2.0) + pow(fabs(handle_point[1] - center_point.y), 2.0));
  float heading = asin((handle_point[1] - center_point.y)/handle_final_distance);
  float handle_button_distance = sqrt(pow(fabs(handle_point[0] - button_point[0]), 2.0) + pow(fabs(handle_point[1] - button_point[1]), 2.0));
  float panel_orient = asin((button_point[1] - handle_point[1])/handle_button_distance);
  if(handle_point[0] - center_point.x <= 0)
  {
    if(handle_point[1] - center_point.y <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  int num_angle = round(heading / M_PI_4);
  heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", heading);
  if(button_point[0] - handle_point[0] <= 0)
  {
    if(button_point[1] - handle_point[1] <= 0)
    {
      panel_orient = -M_PI-panel_orient;
    }
    else
    {
      panel_orient = M_PI-panel_orient;
    }
  }
  float relative_panel_orient = panel_orient - heading;
  ROS_INFO("Raw relative panel orient: %f", relative_panel_orient);
  if(relative_panel_orient < -M_PI)
  {
    relative_panel_orient += 2*M_PI;
  }
  else if(relative_panel_orient > M_PI)
  {
    relative_panel_orient -= 2*M_PI;
  }
  ROS_INFO("Relative panel orient: %f", relative_panel_orient);
  ROS_INFO("XYZ location:\t%f\t%f\t%f\tHeading: %f", handle_point[0], handle_point[1], handle_point[2], heading - yaw);
  
  if((relative_panel_orient < -0.2 && relative_panel_orient > -1.8) || (relative_panel_orient < 2.94 && relative_panel_orient > 1.37))
  {
    // Set unique ids to feature used messages
    trajectory_list_msg.unique_id = 1;
    left_arm_msg.unique_id = 0;
    right_arm_msg.unique_id = 0;
    left_hand_msg.unique_id = 0;
    right_hand_msg.unique_id = 0;
    chest_msg.unique_id = 1;
    pelvis_msg.unique_id = 0;
    left_foot_msg.unique_id = 0;
    right_foot_msg.unique_id = 0;
    
    geometry_msgs::TransformStamped pelvis = tfBuffer->lookupTransform(WORLD, PELVIS, ros::Time(0), ros::Duration(0.2));
    tf2::Quaternion last_heading(pelvis.transform.rotation.x, pelvis.transform.rotation.y, pelvis.transform.rotation.z, pelvis.transform.rotation.w);
    tf2Scalar pelvis_roll;
    tf2Scalar pelvis_pitch;
    tf2Scalar pelvis_yaw;
    tf2::Matrix3x3 pelvis_rotation(last_heading);
    pelvis_rotation.getRPY(pelvis_roll, pelvis_pitch, pelvis_yaw);
    pelvis_rotation.setRPY(0.0, 0.0, pelvis_yaw);
    
    chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
    std::vector<float> new_chest_orient (3, 0);
    std::vector<float> new_chest_ang_vel (3, 0);
    new_chest_orient[1] = 0.4;
    new_chest_orient[2] = pelvis_yaw;
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
    // Move to location
    std::vector<float> end_point(3, 0);
    end_point[0] = -0.60;
    end_point[1] = 0.0;
    end_point[2] = 0.0;
    tf2::Matrix3x3 rotation;
    rotation.setRPY(0.0, 0.0, heading);
    
    x_vector distance;
    distance.x = end_point[0]*rotation[0][0] + end_point[1]*rotation[0][1] + end_point[2]*rotation[0][2] + handle_point[0];
    distance.y = end_point[0]*rotation[1][0] + end_point[1]*rotation[1][1] + end_point[2]*rotation[1][2] + handle_point[1];
    distance.z = end_point[0]*rotation[2][0] + end_point[1]*rotation[2][1] + end_point[2]*rotation[2][2] + handle_point[2];
    
    ihmc_msgs::FootstepDataListRosMessage step_list_msg;
    step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading - yaw);
    step_list_msg = gait_generation.createLowerBodyLinearGaitQueue(step_list_msg, distance, 0.0);
    step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
    x_publisher.publish(step_list_msg);
    
    alignThePanel(tfBuffer, map);
  
    // Move to location
    distance.x = -0.08;
    distance.y = 0.0;
    distance.z = 0.0;
    step_list_msg = gait_generation.createLowerBodyLinearGaitOverride(distance, 0.0);
    step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
    x_publisher.publish(step_list_msg);
  }
  else
  {
    // Move to location
    std::vector<float> end_point(3, 0);
    end_point[0] = -0.68;
    end_point[1] = 0.0;
    end_point[2] = 0.0;
    tf2::Matrix3x3 rotation;
    rotation.setRPY(0.0, 0.0, heading);
    
    x_vector distance;
    distance.x = end_point[0]*rotation[0][0] + end_point[1]*rotation[0][1] + end_point[2]*rotation[0][2] + handle_point[0];
    distance.y = end_point[0]*rotation[1][0] + end_point[1]*rotation[1][1] + end_point[2]*rotation[1][2] + handle_point[1];
    distance.z = end_point[0]*rotation[2][0] + end_point[1]*rotation[2][1] + end_point[2]*rotation[2][2] + handle_point[2];
    
    ihmc_msgs::FootstepDataListRosMessage step_list_msg;
    step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading - yaw);
    step_list_msg = gait_generation.createLowerBodyLinearGaitQueue(step_list_msg, distance, 0.0);
    step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
    x_publisher.publish(step_list_msg);
  }
  map_scan->lowTaskTwoHandleScan(map);
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
    {
      button_point[0] = map->points[point_id].x;
      button_point[1] = map->points[point_id].y;
      button_point[2] = map->points[point_id].z;
    }
    else if(map->points[point_id].g == 0 && map->points[point_id].r == 0 && map->points[point_id].b == 200)
    {
      handle_point[0] = map->points[point_id].x;
      handle_point[1] = map->points[point_id].y;
      handle_point[2] = map->points[point_id].z;
    }
  }
  // Set unique ids to feature used messages
  trajectory_list_msg.unique_id = 1;
  left_arm_msg.unique_id = 1;
  right_arm_msg.unique_id = 0;
  left_hand_msg.unique_id = 0;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 1;
  pelvis_msg.unique_id = 1;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  
  handle_final_distance = sqrt(pow(fabs(handle_point[0] - center_point.x), 2.0) + pow(fabs(handle_point[1] - center_point.y), 2.0));
  heading = asin((handle_point[1] - center_point.y)/handle_final_distance);
  handle_button_distance = sqrt(pow(fabs(handle_point[0] - button_point[0]), 2.0) + pow(fabs(handle_point[1] - button_point[1]), 2.0));
  panel_orient = asin((button_point[1] - handle_point[1])/handle_button_distance);
  if(handle_point[0] - center_point.x <= 0)
  {
    if(handle_point[1] - center_point.y <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  num_angle = round(heading / M_PI_4);
  heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", heading);
  if(button_point[0] - handle_point[0] <= 0)
  {
    if(button_point[1] - handle_point[1] <= 0)
    {
      panel_orient = -M_PI-panel_orient;
    }
    else
    {
      panel_orient = M_PI-panel_orient;
    }
  }
  relative_panel_orient = panel_orient - heading;
  ROS_INFO("Raw relative panel orient: %f", relative_panel_orient);
  if(relative_panel_orient < -M_PI)
  {
    relative_panel_orient += 2*M_PI;
  }
  else if(relative_panel_orient > M_PI)
  {
    relative_panel_orient -= 2*M_PI;
  }
  ROS_INFO("Relative panel orient: %f", relative_panel_orient);
  ROS_INFO("XYZ location:\t%f\t%f\t%f\tHeading: %f", handle_point[0], handle_point[1], handle_point[2], heading - yaw);
  
  std::vector<float> left_arm_swing = grabPanelTrajectory(LEFT, 0);
  std::vector<float> ang_vel(7, 0);
  if(relative_panel_orient < M_PI_2 && relative_panel_orient > -M_PI_2)
  {
    left_arm_swing[4] += M_PI;
  }
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, ang_vel, left_arm_msg);
  geometry_msgs::TransformStamped pelvis = tfBuffer->lookupTransform(WORLD, PELVIS, ros::Time(0), ros::Duration(0.2));
  
  tf2::Quaternion last_heading(pelvis.transform.rotation.x, pelvis.transform.rotation.y, pelvis.transform.rotation.z, pelvis.transform.rotation.w);
  tf2Scalar pelvis_roll;
  tf2Scalar pelvis_pitch;
  tf2Scalar pelvis_yaw;
  tf2::Matrix3x3 pelvis_rotation(last_heading);
  pelvis_rotation.getRPY(pelvis_roll, pelvis_pitch, pelvis_yaw);
  pelvis_rotation.setRPY(0.0, 0.0, pelvis_yaw);
  
  chest_msg.taskspace_trajectory_points.clear();
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.4;
  new_chest_orient[2] = pelvis_yaw - 0.3;
  float chest_time = 1.0;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(1.0, new_chest_orient, new_chest_ang_vel));
  
  pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(0.5, 0.875));
  
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
}

void
Solar_Components_Driver::alignThePanel(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  X_Publisher x_publisher;
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
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
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
  
  std::vector<float> button_point(3, 0);
  std::vector<float> handle_point(3, 0);
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
    {
      button_point[0] = map->points[point_id].x;
      button_point[1] = map->points[point_id].y;
      button_point[2] = map->points[point_id].z;
    }
    else if(map->points[point_id].g == 0 && map->points[point_id].r == 0 && map->points[point_id].b == 200)
    {
      handle_point[0] = map->points[point_id].x;
      handle_point[1] = map->points[point_id].y;
      handle_point[2] = map->points[point_id].z;
    }
  }
  
  float handle_final_distance = sqrt(pow(fabs(handle_point[0] - center_point.x), 2.0) + pow(fabs(handle_point[1] - center_point.y), 2.0));
  float heading = asin((handle_point[1] - center_point.y)/handle_final_distance);
  float handle_button_distance = sqrt(pow(fabs(handle_point[0] - button_point[0]), 2.0) + pow(fabs(handle_point[1] - button_point[1]), 2.0));
  float panel_orient = asin((button_point[1] - handle_point[1])/handle_button_distance);
  if(handle_point[0] - center_point.x <= 0)
  {
    if(handle_point[1] - center_point.y <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  int num_angle = round(heading / M_PI_4);
  heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", heading);
  if(button_point[0] - handle_point[0] <= 0)
  {
    if(button_point[1] - handle_point[1] <= 0)
    {
      panel_orient = -M_PI-panel_orient;
    }
    else
    {
      panel_orient = M_PI-panel_orient;
    }
  }
  float relative_panel_orient = panel_orient - heading;
  ROS_INFO("Raw relative panel orient: %f", relative_panel_orient);
  if(relative_panel_orient < -M_PI)
  {
    relative_panel_orient += 2*M_PI;
  }
  else if(relative_panel_orient > M_PI)
  {
    relative_panel_orient -= 2*M_PI;
  }
  ROS_INFO("Relative panel orient: %f", relative_panel_orient);
  ROS_INFO("XYZ location:\t%f\t%f\t%f\tHeading: %f", handle_point[0], handle_point[1], handle_point[2], heading - yaw);
  
  float orient_offset;
  if(relative_panel_orient < -0.2 && relative_panel_orient > -1.8)
  {
    orient_offset = relative_panel_orient+1.5;
  }
  else if(relative_panel_orient < 2.94 && relative_panel_orient > 1.37)
  {
    orient_offset = relative_panel_orient-1.8;
  }
  
  std::vector<float> left_arm_swing = alignPanelTrajectory(LEFT, 0);
  std::vector<float> right_arm_swing = alignPanelTrajectory(RIGHT, 0);
  std::vector<float> ang_vel(7, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, ang_vel, left_arm_msg);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, ang_vel, right_arm_msg);
  left_arm_swing = alignPanelTrajectory(LEFT, 1);
  right_arm_swing = alignPanelTrajectory(RIGHT, 1);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(2.0, left_arm_swing, ang_vel, left_arm_msg);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(2.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = alignPanelTrajectory(RIGHT, 2);
  right_arm_swing[2] += orient_offset/2;
  right_arm_swing[4] += orient_offset/2;
  if(orient_offset > 0)
  {
    right_arm_swing[3] += orient_offset/8;
  }
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(3.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(3.2, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = alignPanelTrajectory(RIGHT, 3);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(4.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = alignPanelTrajectory(RIGHT, 4);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(6.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = alignPanelTrajectory(RIGHT, 5);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(7.0, right_arm_swing, ang_vel, right_arm_msg);
  left_arm_swing = alignPanelTrajectory(LEFT, 0);
  right_arm_swing = alignPanelTrajectory(RIGHT, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(8.0, left_arm_swing, ang_vel, left_arm_msg);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(8.0, right_arm_swing, ang_vel, right_arm_msg);
  
  geometry_msgs::TransformStamped pelvis = tfBuffer->lookupTransform(WORLD, PELVIS, ros::Time(0), ros::Duration(0.2));
  
  tf2::Quaternion last_heading(pelvis.transform.rotation.x, pelvis.transform.rotation.y, pelvis.transform.rotation.z, pelvis.transform.rotation.w);
  tf2Scalar pelvis_roll;
  tf2Scalar pelvis_pitch;
  tf2Scalar pelvis_yaw;
  tf2::Matrix3x3 pelvis_rotation(last_heading);
  pelvis_rotation.getRPY(pelvis_roll, pelvis_pitch, pelvis_yaw);
  pelvis_rotation.setRPY(0.0, 0.0, pelvis_yaw);
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.4;
  new_chest_orient[2] = pelvis_yaw;
  float chest_time = 1.0;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(1.0, new_chest_orient, new_chest_ang_vel));
  
  pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(0.5, 1.06));
  
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
  finger_pos[0] = 0.5;
  finger_pos[1] = -0.6;
  finger_pos[2] = -0.2;
  finger_pos[3] = -0.2;
  finger_pos[4] = -0.2;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
  std_msgs::Float64MultiArray right_fingers;
  finger_pos[0] = 1.5;
  finger_pos[1] = 0.0;
  finger_pos[2] = 0.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(3.0).sleep();
  finger_pos[0] = 1.5;
  finger_pos[1] = 1.0;
  finger_pos[2] = 1.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(2.5).sleep();
  finger_pos[0] = 1.5;
  finger_pos[1] = 0.0;
  finger_pos[2] = 0.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(2.0).sleep();
  finger_pos[0] = 0.5;
  finger_pos[1] = 0.6;
  finger_pos[2] = 0.2;
  finger_pos[3] = 0.2;
  finger_pos[4] = 0.2;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(0.5).sleep();
}

std::vector<float>
Solar_Components_Driver::alignPanelTrajectory(bool robot_side_arm, int step_num)
{
  std::vector<float> trajectory_out(7, 0);
  if(step_num == 0)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = -1.33;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -1.7;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = 1.33;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = 1.7;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 1)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.2;
      trajectory_out[1] = -1.33;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -1.3;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.3;
      trajectory_out[1] = 1.45;
      trajectory_out[2] = 0.5;
      trajectory_out[3] = 0.8;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 2)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.2;
      trajectory_out[1] = -1.5;
      trajectory_out[2] = 0.4;
      trajectory_out[3] = -1.3;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.1;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 1.5;
      trajectory_out[3] = 0.7;
      trajectory_out[4] = 1.9;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 3)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.1;
      trajectory_out[1] = -1.5;
      trajectory_out[2] = 0.7;
      trajectory_out[3] = -0.5;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.6;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 1.5;
      trajectory_out[3] = 0.8;
      trajectory_out[4] = 0.6;
      trajectory_out[5] = 0.6;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 4)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.0;
      trajectory_out[1] = -1.33;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -0.5;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.2;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 1.5;
      trajectory_out[3] = 0.8;
      trajectory_out[4] = 0.6;
      trajectory_out[5] = 0.6;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 5)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.0;
      trajectory_out[1] = -1.33;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -0.5;
      trajectory_out[4] = 1.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.1;
      trajectory_out[1] = 1.35;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = 0.6;
      trajectory_out[4] = 1.8;
      trajectory_out[5] = 0.6;
      trajectory_out[6] = 0.0;
    }
  }
  return trajectory_out;
}

void
Solar_Components_Driver::grabPanel(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  X_Publisher x_publisher;
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
  
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
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
  left_hand_msg.unique_id = 1;
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
  left_hand_msg.base_for_control = ihmc_msgs::HandTrajectoryRosMessage::WORLD;
  
  std::vector<float> button_point(3, 0);
  std::vector<float> handle_point(3, 0);
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
    {
      button_point[0] = map->points[point_id].x;
      button_point[1] = map->points[point_id].y;
      button_point[2] = map->points[point_id].z;
    }
    else if(map->points[point_id].g == 0 && map->points[point_id].r == 0 && map->points[point_id].b == 200)
    {
      handle_point[0] = map->points[point_id].x;
      handle_point[1] = map->points[point_id].y;
      handle_point[2] = map->points[point_id].z;
    }
  }
  
  float handle_final_distance = sqrt(pow(fabs(handle_point[0] - center_point.x), 2.0) + pow(fabs(handle_point[1] - center_point.y), 2.0));
  float heading = asin((handle_point[1] - center_point.y)/handle_final_distance);
  if(handle_point[0] - center_point.x <= 0)
  {
    if(handle_point[1] - center_point.y <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  float handle_button_distance = sqrt(pow(handle_point[0] - button_point[0], 2.0) + pow(handle_point[1] - button_point[1], 2.0));
  float panel_orient = asin((button_point[1] - handle_point[1])/handle_button_distance);
  ROS_INFO("heading: %f", heading);
  if(button_point[0] - handle_point[0] <= 0)
  {
    if(button_point[1] - handle_point[1] <= 0)
    {
      panel_orient = -M_PI-panel_orient;
    }
    else
    {
      panel_orient = M_PI-panel_orient;
    }
  }
  ROS_INFO("panel_orient: %f", panel_orient);
  ROS_INFO("button xy: %f\t%f", button_point[0], button_point[1]);
  ROS_INFO("handle xy: %f\t%f", handle_point[0], handle_point[1]);
  ROS_INFO("panel_distance: %f", handle_button_distance);
  
  std::vector<float> pulled_hand(3, 0);
  pulled_hand[0] = 0.0;
  pulled_hand[1] = 0.2;
  std::vector<float> pushed_hand(3, 0);
  pushed_hand[0] = 0.0;
  pushed_hand[1] = -0.1;

  float relative_panel_orient = panel_orient - heading;
  if(relative_panel_orient < -M_PI)
  {
    relative_panel_orient += 2*M_PI;
  }
  else if(relative_panel_orient > M_PI)
  {
    relative_panel_orient -= 2*M_PI;
  }
  ROS_INFO("Relative panel orient: %f", relative_panel_orient);
  tf2::Matrix3x3 rotation;
  if(relative_panel_orient < M_PI_2 && relative_panel_orient > -M_PI_2)
  {
    rotation.setRPY(0.0, 0.0, panel_orient);
  }
  else
  {
    if(relative_panel_orient > M_PI_2)
    {
      rotation.setRPY(0.0, 0.0, panel_orient - M_PI);
    }
    else
    {
      rotation.setRPY(0.0, 0.0, panel_orient + M_PI);
    }
  }
  // Check if the points are behind valkyrie, thus prompting a rotation offset
  std::vector<float> point_comparison(3, 0);
  
  std::vector<float> pos(3, 0);
  std::vector<float> orient(3, 0);
  std::vector<float> line_vel(3, 0);
  std::vector<float> ang_vel(3, 0);
  if(relative_panel_orient < M_PI_2 && relative_panel_orient > -M_PI_2)
  {
    point_comparison[0] = pulled_hand[0]*rotation[0][0] + pulled_hand[1]*rotation[0][1];
    point_comparison[1] = pulled_hand[0]*rotation[1][0] + pulled_hand[1]*rotation[1][1];
    pulled_hand[0] = point_comparison[0];
    pulled_hand[1] = point_comparison[1];
    pos[0] = handle_point[0] + pulled_hand[0];
    pos[1] = handle_point[1] + pulled_hand[1];
    pos[2] = 0.85 + center_point.z;
    orient[0] = 0.0;
    orient[1] = 1.57;
    orient[2] = panel_orient - M_PI;
    
    point_comparison[0] = pushed_hand[0]*rotation[0][0] + pushed_hand[1]*rotation[0][1];
    point_comparison[1] = pushed_hand[0]*rotation[1][0] + pushed_hand[1]*rotation[1][1];
    pushed_hand[0] = point_comparison[0];
    pushed_hand[1] = point_comparison[1];
    left_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(1.0, pos, orient, line_vel, ang_vel));
    left_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(4.0, pos, orient, line_vel, ang_vel));
    ros::Duration(1.0).sleep();
    pos[0] = handle_point[0] + pushed_hand[0];
    pos[1] = handle_point[1] + pushed_hand[1];
    pos[2] = 0.85 + center_point.z;
    orient[0] = 0.0;
    orient[1] = 1.57;
    orient[2] = panel_orient - M_PI;
    
    left_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(6.0, pos, orient, line_vel, ang_vel));
  }
  else
  {
    point_comparison[0] = pulled_hand[0]*rotation[0][0] + pulled_hand[1]*rotation[0][1];
    point_comparison[1] = pulled_hand[0]*rotation[1][0] + pulled_hand[1]*rotation[1][1];
    pulled_hand[0] = point_comparison[0];
    pulled_hand[1] = point_comparison[1];
    pos[0] = handle_point[0] + pulled_hand[0];
    pos[1] = handle_point[1] + pulled_hand[1];
    pos[2] = 0.78 + center_point.z;
    orient[0] = 0.0;
    orient[1] = -1.57;
    orient[2] = panel_orient;
    
    point_comparison[0] = pushed_hand[0]*rotation[0][0] + pushed_hand[1]*rotation[0][1];
    point_comparison[1] = pushed_hand[0]*rotation[1][0] + pushed_hand[1]*rotation[1][1];
    pushed_hand[0] = point_comparison[0];
    pushed_hand[1] = point_comparison[1];
    left_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(1.0, pos, orient, line_vel, ang_vel));
    left_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(4.0, pos, orient, line_vel, ang_vel));
    ros::Duration(1.0).sleep();
    pos[0] = handle_point[0] + pushed_hand[0];
    pos[1] = handle_point[1] + pushed_hand[1];
    pos[2] = 0.78 + center_point.z;
    orient[0] = 0.0;
    orient[1] = -1.57;
    orient[2] = panel_orient;
    
    left_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(6.0, pos, orient, line_vel, ang_vel));
  }
  
  pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(0.5, 0.87));
  
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
  
  ros::Duration(1.0).sleep();
  std_msgs::Float64MultiArray left_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 1.5;
  finger_pos[1] = -0.4;
  finger_pos[2] = -0.0;
  finger_pos[3] = -0.0;
  finger_pos[4] = -0.0;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
  
  ros::Duration(5.0).sleep();
  finger_pos[0] = 1.5;
  finger_pos[1] = -0.8;
  finger_pos[2] = -0.8;
  finger_pos[3] = -0.8;
  finger_pos[4] = -0.8;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
}

void
Solar_Components_Driver::carryPanel()
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
  
  std::vector<float> left_arm_swing = pickUpPanelTrajectory(LEFT, 0);
  std::vector<float> ang_vel(7, 0);
  //left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = pickUpPanelTrajectory(LEFT, 1);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(3.0, left_arm_swing, ang_vel, left_arm_msg);
  
  std::vector<float> right_arm_swing = pickUpPanelTrajectory(RIGHT, 0);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(5.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = pickUpPanelTrajectory(RIGHT, 1);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(6.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = pickUpPanelTrajectory(RIGHT, 2);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(7.0, right_arm_swing, ang_vel, right_arm_msg);
  
  pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(7.0, 1.06));
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.1;
  new_chest_orient[2] = yaw;
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
  std_msgs::Float64MultiArray left_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 1.5;
  finger_pos[1] = -1.0;
  finger_pos[2] = -0.8;
  finger_pos[3] = -0.8;
  finger_pos[4] = -0.8;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
  std_msgs::Float64MultiArray right_fingers;
  finger_pos[0] = 0.0;
  finger_pos[1] = 0.0;
  finger_pos[2] = 0.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(7.0).sleep();
  finger_pos[0] = 0.0;
  finger_pos[1] = 0.0;
  finger_pos[2] = 0.6;
  finger_pos[3] = 0.6;
  finger_pos[4] = 0.6;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(0.5).sleep();
}

std::vector<float>
Solar_Components_Driver::grabPanelTrajectory(bool robot_side_arm, int step_num)
{
  std::vector<float> trajectory_out(7, 0);
  if(step_num == 0)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.1;
      trajectory_out[1] = -0.8;
      trajectory_out[2] = 0.1;
      trajectory_out[3] = -1.7;
      trajectory_out[4] = -0.7;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = 1.47;
      trajectory_out[2] = -0.8;
      trajectory_out[3] = 1.57;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.60;
      trajectory_out[6] = 0.28;
    }
  }
  return trajectory_out;
}

std::vector<float>
Solar_Components_Driver::pickUpPanelTrajectory(bool robot_side_arm, int step_num)
{
  std::vector<float> trajectory_out(7, 0);
  if(step_num == 0)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.3;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -2.3;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.28;
    }
    else
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = 1.47;
      trajectory_out[2] = -0.2;
      trajectory_out[3] = 1.57;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 1)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -2.3;
      trajectory_out[1] = -1.3;
      trajectory_out[2] = 2.16;
      trajectory_out[3] = -2.0;
      trajectory_out[4] = -0.4;
      trajectory_out[5] = -0.6;
      trajectory_out[6] = -0.2;
    }
    else
    {
      trajectory_out[0] = -1.8;
      trajectory_out[1] = 1.3;
      trajectory_out[2] = 1.6;
      trajectory_out[3] = 0.4;
      trajectory_out[4] = 1.0;
      trajectory_out[5] = 0.60;
      trajectory_out[6] = 0.28;
    }
  }
  if(step_num == 2)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -2.1;
      trajectory_out[1] = -1.2;
      trajectory_out[2] = 2.0;
      trajectory_out[3] = -2.0;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = -0.6;
      trajectory_out[6] = -0.0;
    }
    else
    {
      trajectory_out[0] = -0.6;
      trajectory_out[1] = 1.5;
      trajectory_out[2] = 1.8;
      trajectory_out[3] = 1.4;
      trajectory_out[4] = 0.8;
      trajectory_out[5] = 0.60;
      trajectory_out[6] = 0.28;
    }
  }
  return trajectory_out;
}

std::vector<float>
Solar_Components_Driver::dropPanelTrajectory(bool robot_side_arm, int step_num)
{
  std::vector<float> trajectory_out(7, 0);
  if(step_num == 0)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.8;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = -0.5;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = -0.6;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = 1.47;
      trajectory_out[2] = -0.8;
      trajectory_out[3] = 1.8;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.60;
      trajectory_out[6] = 0.28;
    }
  }
  if(step_num == 1)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.56;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = -0.5;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = -0.6;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.57;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = 0.5;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 2)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.8;
      trajectory_out[1] = -1.2;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = -0.0;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.0;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = 0.8;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.2;
    }
  }
  if(step_num == 3)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.4;
      trajectory_out[1] = -1.32;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = -0.0;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.57;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = 0.5;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 4)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.4;
      trajectory_out[1] = -1.35;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = -0.3;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = -0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.57;
      trajectory_out[1] = 1.51;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = 0.5;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  return trajectory_out;
}

std::vector<float>
Solar_Components_Driver::grabCableTrajectory(bool robot_side_arm, int step_num)
{
  std::vector<float> trajectory_out(7, 0);
  if(step_num == 0)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.7;
      trajectory_out[1] = -1.0;
      trajectory_out[2] = -0.6;
      trajectory_out[3] = -0.2;
      trajectory_out[4] = 2.5;
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
      trajectory_out[0] = -1.3;
      trajectory_out[1] = 1.38;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = 0.0;
      trajectory_out[4] = 1.57;
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
      trajectory_out[0] = -0.4;
      trajectory_out[1] = 0.0;
      trajectory_out[2] = 2.0;
      trajectory_out[3] = 2.0;
      trajectory_out[4] = 1.9;
      trajectory_out[5] = 0.4;
      trajectory_out[6] = 0.0;
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
      trajectory_out[2] = 1.7;
      trajectory_out[3] = 0.3;
      trajectory_out[4] = 1.0;
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
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -1.8;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -2.5;
      trajectory_out[1] = 0.6;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = 1.3;
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
      trajectory_out[0] = 0.9;
      trajectory_out[1] = 1.3;
      trajectory_out[2] = -1.5;
      trajectory_out[3] = 1.0;
      trajectory_out[4] = 2.0;
      trajectory_out[5] = 0.3;
      trajectory_out[6] = 0.35;
    }
  }
  if(step_num == 7)
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
      trajectory_out[0] = 0.9;
      trajectory_out[1] = 1.3;
      trajectory_out[2] = -1.5;
      trajectory_out[3] = 0.9;
      trajectory_out[4] = 1.6;
      trajectory_out[5] = 0.3;
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
      trajectory_out[2] = 1.3;
      trajectory_out[3] = 1.3;
      trajectory_out[4] = -1.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
  }
  return trajectory_out;
}

void
Solar_Components_Driver::returnToPath(std::vector<float> next_point)
{
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  X_Publisher x_publisher;
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
  
  float next_point_distance = sqrt(pow(fabs(next_point[0] - center_point.x), 2.0) + pow(fabs(next_point[1] - center_point.y), 2.0));
  float heading = asin((next_point[1] - center_point.y)/next_point_distance);
  
  if(next_point[0] - center_point.x <= 0)
  {
    if(next_point[1] - center_point.y <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  x_vector distance;
  distance.x = -0.2;
  distance.y = -0.0;
  distance.z = -0.0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOverride(distance, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  ROS_INFO("Raw Heading: %f", heading);
  ROS_INFO("Combined Heading: %f", heading-yaw);
  ROS_INFO("New checkpoint found XYZ:\t%f\t%f\t%f", next_point[0], next_point[1], next_point[2]);
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

void
Solar_Components_Driver::alignToArray(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  X_Publisher x_publisher;
  geometry_msgs::TransformStamped torso = tfBuffer->lookupTransform(WORLD, TORSO, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped left_wrist = tfBuffer->lookupTransform(WORLD, LEFT_WRIST, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_wrist = tfBuffer->lookupTransform(WORLD, RIGHT_WRIST, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer->lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  
  float hand_seperation = sqrt(pow(fabs(left_wrist.transform.translation.x-right_wrist.transform.translation.x), 2.0) + pow(fabs(left_wrist.transform.translation.y-right_wrist.transform.translation.y), 2.0));
  float hand_heading = asin((left_wrist.transform.translation.y-right_wrist.transform.translation.y)/hand_seperation);
  if(left_wrist.transform.translation.x-right_wrist.transform.translation.x < 0)
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
  ROS_INFO("headings:\t%f\t%f", hand_heading, yaw);
  int num_angle = round(yaw / M_PI_4);
  hand_heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", hand_heading);
  if(hand_heading < -M_PI || hand_heading > M_PI)
  {
    if(hand_heading > 0)
    {
      hand_heading -= 2*M_PI;
    }
    else
    {
      hand_heading += 2*M_PI;
    }
  }
  ros::Duration(0.01).sleep();
	float final_heading = hand_heading - yaw;
  if(final_heading < -M_PI || final_heading > M_PI)
  {
    if(final_heading > 0)
    {
      final_heading -= 2*M_PI;
    }
    else
    {
      final_heading += 2*M_PI;
    }
  }
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(final_heading);
  x_vector destination;
  destination.x = 0.05;
  destination.y = 0;
  destination.z = 0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOffsetQueue(step_list_msg, destination, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  x_publisher.publish(step_list_msg);
}

std::vector<float>
Solar_Components_Driver::grabTableTrajectory(bool robot_side_arm, int step_num)
{
  std::vector<float> trajectory_out(7, 0);
  if(step_num == 0)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = -1.4;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -0.35;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -0.0;
      trajectory_out[1] = 1.4;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = 0.35;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  if(step_num == 1)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.8;
      trajectory_out[1] = -1.47;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -0.35;
      trajectory_out[4] = -0.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.4;
    }
    else
    {
      trajectory_out[0] = -0.8;
      trajectory_out[1] = 1.47;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = 0.35;
      trajectory_out[4] = -0.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.4;
    }
  }
  if(step_num == 2)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -0.6;
      trajectory_out[1] = -1.25;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = -0.35;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.4;
    }
    else
    {
      trajectory_out[0] = -0.6;
      trajectory_out[1] = 1.25;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 0.35;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.4;
    }
  }
  if(step_num == 7)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = 0.0;
      trajectory_out[1] = -0.0;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -1.0;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = 0.0;
      trajectory_out[1] = 0.0;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = 1.0;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
  }
  return trajectory_out;
}

void
Solar_Components_Driver::dropPanel()
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
  
  std::vector<float> left_arm_swing = dropPanelTrajectory(LEFT, 1);
  std::vector<float> ang_vel(7, 0);
  //left_arm_msg = trajectory_generation.appendTrajectoryPoint(0.5, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = pickUpPanelTrajectory(LEFT, 1);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = dropPanelTrajectory(LEFT, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(3.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(6.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = dropPanelTrajectory(LEFT, 1);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(7.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = dropPanelTrajectory(LEFT, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(8.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = dropPanelTrajectory(LEFT, 2);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(9.0, left_arm_swing, ang_vel, left_arm_msg);
  
  std::vector<float> right_arm_swing = dropPanelTrajectory(RIGHT, 0);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, ang_vel, right_arm_msg);
  
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
  finger_pos[1] = -2.0;
  finger_pos[2] = -2.0;
  finger_pos[3] = -2.0;
  finger_pos[4] = -2.0;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
  std_msgs::Float64MultiArray right_fingers;
  finger_pos[0] = 0.0;
  finger_pos[1] = 0.0;
  finger_pos[2] = 0.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(7.0).sleep();
  finger_pos[0] = 1.5;
  finger_pos[1] = -0.0;
  finger_pos[2] = -0.0;
  finger_pos[3] = -0.0;
  finger_pos[4] = -0.0;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
  ros::Duration(3.0).sleep();
}

std::vector<float>
Solar_Components_Driver::deployPanelTrajectory(bool robot_side_arm, int step_num)
{
  std::vector<float> trajectory_out(7, 0);
  if(step_num == 0)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = 0.0;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 0.0;
      trajectory_out[3] = -1.8;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.2;
    }
    else
    {
      trajectory_out[0] = 0.3;
      trajectory_out[1] = 1.5;
      trajectory_out[2] = -0.3;
      trajectory_out[3] = 2.0;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
  }
  if(step_num == 1)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.3;
      trajectory_out[1] = -1.51;
      trajectory_out[2] = 1.6;
      trajectory_out[3] = -0.4;
      trajectory_out[4] = 0.2;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.2;
    }
    else
    {
      trajectory_out[0] = -1.1;
      trajectory_out[1] = 1.3;
      trajectory_out[2] = -0.1;
      trajectory_out[3] = 0.2;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
  }
  if(step_num == 2)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.8;
      trajectory_out[1] = -1.0;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = -1.1;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = 0.3;
      trajectory_out[1] = 1.1;
      trajectory_out[2] = -0.9;
      trajectory_out[3] = 1.5;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
  }
  if(step_num == 3)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -2.0;
      trajectory_out[1] = -1.3;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = -0.9;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.2;
      trajectory_out[1] = 1.5;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 0.0;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
  }
  if(step_num == 4)
  {
    if(robot_side_arm == LEFT)
    {
      trajectory_out[0] = -1.4;
      trajectory_out[1] = -1.3;
      trajectory_out[2] = 1.57;
      trajectory_out[3] = -1.2;
      trajectory_out[4] = 1.57;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = 0.0;
    }
    else
    {
      trajectory_out[0] = -1.2;
      trajectory_out[1] = 1.5;
      trajectory_out[2] = 0.2;
      trajectory_out[3] = 0.0;
      trajectory_out[4] = 0.0;
      trajectory_out[5] = 0.0;
      trajectory_out[6] = -0.2;
    }
  }
  return trajectory_out;
}

void
Solar_Components_Driver::deployPanel(Map_Scan* map_scan, pcl::PointCloud<pcl::PointXYZRGB>* map)
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
  std::vector<float> ang_vel(7, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(0.5, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = deployPanelTrajectory(LEFT, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = deployPanelTrajectory(LEFT, 1);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(4.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = deployPanelTrajectory(LEFT, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(5.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = deployPanelTrajectory(LEFT, 2);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(6.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = deployPanelTrajectory(LEFT, 3);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(7.0, left_arm_swing, ang_vel, left_arm_msg);
  //left_arm_swing = deployPanelTrajectory(LEFT, 4);
  //left_arm_msg = trajectory_generation.appendTrajectoryPoint(8.0, left_arm_swing, ang_vel, left_arm_msg);
  //left_arm_swing = deployPanelTrajectory(LEFT, 3);
  //left_arm_msg = trajectory_generation.appendTrajectoryPoint(9.0, left_arm_swing, ang_vel, left_arm_msg);
  
  std::vector<float> right_arm_swing = grabTableTrajectory(RIGHT, 7);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(0.5, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = deployPanelTrajectory(RIGHT, 0);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = deployPanelTrajectory(RIGHT, 1);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(4.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = deployPanelTrajectory(RIGHT, 0);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(5.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = deployPanelTrajectory(RIGHT, 2);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(6.0, right_arm_swing, ang_vel, right_arm_msg);
  
  chest_msg.execution_mode = ihmc_msgs::ChestTrajectoryRosMessage::OVERRIDE;
  std::vector<float> new_chest_orient (3, 0);
  std::vector<float> new_chest_ang_vel (3, 0);
  new_chest_orient[1] = 0.2;
  new_chest_orient[2] = yaw + 0.6;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(1.0, new_chest_orient, new_chest_ang_vel));
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(6.0, new_chest_orient, new_chest_ang_vel));
  new_chest_orient[1] = 0.2;
  new_chest_orient[2] = yaw + 0.3;
  chest_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSO3TrajectoryOrient(7.0, new_chest_orient, new_chest_ang_vel));
  
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
  finger_pos[0] = 0.0;
  finger_pos[1] = 0.0;
  finger_pos[2] = 0.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(4.0).sleep();
  finger_pos[0] = 0.0;
  finger_pos[1] = -1.0;
  finger_pos[2] = -1.0;
  finger_pos[3] = -1.0;
  finger_pos[4] = -1.0;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
  ros::Duration(3.0).sleep();
  
  map_scan->simpleTaskTwoScan(map);
  
  left_arm_msg.unique_id = 0;
  right_arm_msg.unique_id = 0;
  left_hand_msg.unique_id = 1;
  right_hand_msg.unique_id = 0;
  chest_msg.unique_id = 0;
  pelvis_msg.unique_id = 0;
  left_foot_msg.unique_id = 0;
  right_foot_msg.unique_id = 0;
  left_hand_msg.base_for_control = ihmc_msgs::HandTrajectoryRosMessage::WORLD;
  
  std::vector<float> button_point(3, 0);
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
    {
      button_point[0] = map->points[point_id].x;
      button_point[1] = map->points[point_id].y;
      button_point[2] = map->points[point_id].z;
    }
  }
  std::vector<float> pos(3, 0);
  std::vector<float> orient(3, 0);
  std::vector<float> line_vel(3, 0);
  std::vector<float> ang_vel_se3(3, 0);
  orient[0] = 0.0;
  orient[1] = 1.57;
  orient[2] = yaw+0.6 - 2.0;
  pos[0] = button_point[0];
  pos[1] = button_point[1];
  pos[2] = 1.1;
  left_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(2.0, pos, orient, line_vel, ang_vel_se3));
  pos[0] = button_point[0];
  pos[1] = button_point[1];
  pos[2] = 1.4;
  left_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(3.0, pos, orient, line_vel, ang_vel_se3));
  
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
  ros::Duration(2.5).sleep();
}

void
Solar_Components_Driver::grabCable()
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
  std::vector<float> ang_vel(7, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, left_arm_swing, ang_vel, left_arm_msg);
  left_arm_swing = grabCableTrajectory(LEFT, 0);
  left_arm_msg = trajectory_generation.appendTrajectoryPoint(3.0, left_arm_swing, ang_vel, left_arm_msg);
  
  std::vector<float> right_arm_swing = dropPanelTrajectory(RIGHT, 0);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(1.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 0);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(2.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 1);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(3.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 2);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(5.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 0);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(6.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 1);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(8.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 2);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(10.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 3);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(11.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 4);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(14.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 6);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(17.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 7);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(18.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 8);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(20.0, right_arm_swing, ang_vel, right_arm_msg);
  right_arm_swing = grabCableTrajectory(RIGHT, 5);
  right_arm_msg = trajectory_generation.appendTrajectoryPoint(21.0, right_arm_swing, ang_vel, right_arm_msg);
  
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
  x_publisher.publish(trajectory_list_msg);
  std_msgs::Float64MultiArray left_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 0.5;
  finger_pos[1] = -0.0;
  finger_pos[2] = -0.2;
  finger_pos[3] = -0.2;
  finger_pos[4] = -0.2;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
  std_msgs::Float64MultiArray right_fingers;
  finger_pos[0] = 0.5;
  finger_pos[1] = 0.4;
  finger_pos[2] = 0.2;
  finger_pos[3] = 0.2;
  finger_pos[4] = 0.2;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(10.0).sleep();
  finger_pos[0] = 1.5;
  finger_pos[1] = 0.9;
  finger_pos[2] = 0.08;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(2.5).sleep();
  finger_pos[0] = 1.5;
  finger_pos[1] = 0.9;
  finger_pos[2] = 0.3;
  finger_pos[3] = 0.2;
  finger_pos[4] = 0.1;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(1.5).sleep();
  finger_pos[0] = 1.3;
  finger_pos[1] = 0.9;
  finger_pos[2] = 0.9;
  finger_pos[3] = 0.2;
  finger_pos[4] = 0.2;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(4.0).sleep();
  finger_pos[0] = 1.3;
  finger_pos[1] = 2.0;
  finger_pos[2] = 2.0;
  finger_pos[3] = 2.0;
  finger_pos[4] = 2.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  ros::Duration(5.0).sleep();
}

void
Solar_Components_Driver::plugCable(Map_Scan* map_scan, tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  X_Publisher x_publisher;
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
  right_hand_msg.unique_id = 1;
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
  right_hand_msg.base_for_control = ihmc_msgs::HandTrajectoryRosMessage::WORLD;
  
  std::vector<float> plug_point(3, 0);
  std::vector<float> cable_point(3, 0);
  
  
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  float heading = yaw + M_PI;
  std::vector<float> base_location(3, 0);
  std::vector<float> tmp_location(3, 0);
  base_location[0] = 0.6;
  base_location[1] = -0.2;
  base_location[2] = 1.1;
  tmp_location[0] = base_location[0]*rotation[0][0] + base_location[1]*rotation[0][1] + base_location[2]*rotation[0][2];
  tmp_location[1] = base_location[0]*rotation[1][0] + base_location[1]*rotation[1][1] + base_location[2]*rotation[1][2];
  tmp_location[2] = base_location[0]*rotation[2][0] + base_location[1]*rotation[2][1] + base_location[2]*rotation[2][2];
  base_location[0] = tmp_location[0] + center_point.x;
  base_location[1] = tmp_location[1] + center_point.y;
  base_location[2] = tmp_location[2] + center_point.z;
  
  std::vector<float> pos(3, 0);
  std::vector<float> orient(3, 0);
  std::vector<float> line_vel(3, 0);
  std::vector<float> ang_vel(3, 0);
  orient[0] = -0.0;
  orient[1] = 0.2;
  orient[2] = heading;
  pos[0] = base_location[0];
  pos[1] = base_location[1];
  pos[2] = base_location[2];
  right_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(2.0, pos, orient, line_vel, ang_vel));
  
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
  ros::Duration(2.0).sleep();
  
  map_scan->lowTaskTwoPlugScan(map);
  
  // Locate points of each point
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 200 && map->points[point_id].r == 255)
    {
      plug_point[0] = map->points[point_id].x;
      plug_point[1] = map->points[point_id].y;
      plug_point[2] = map->points[point_id].z;
    }
    else if(map->points[point_id].g == 0 && map->points[point_id].r == 0 && map->points[point_id].b == 200)
    {
      cable_point[0] = map->points[point_id].x;
      cable_point[1] = map->points[point_id].y;
      cable_point[2] = map->points[point_id].z;
    }
  }
  
  right_hand_msg.taskspace_trajectory_points.clear();
  pos[0] += plug_point[0] - cable_point[0];
  pos[1] += plug_point[1] - cable_point[1];
  right_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(2.0, pos, orient, line_vel, ang_vel));
  
  ROS_INFO("XYZ location:\t%f\t%f\t%f", plug_point[0] - cable_point[0], plug_point[1] - cable_point[1], plug_point[2] - cable_point[2]);
  
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
  ros::Duration(2.0).sleep();
  
  map_scan->lowTaskTwoPlugScan(map);
  
  // Locate points of each point
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].g == 0 && map->points[point_id].r == 0 && map->points[point_id].b == 200)
    {
      cable_point[0] = map->points[point_id].x;
      cable_point[1] = map->points[point_id].y;
      cable_point[2] = map->points[point_id].z;
    }
  }
  
  right_hand_msg.taskspace_trajectory_points.clear();
  pos[0] += plug_point[0] - cable_point[0];
  pos[1] += plug_point[1] - cable_point[1];
  pos[2] -= 0.12;
  
  rotation.setRPY(0.0, 0.0, yaw);
  base_location[0] = 0.02;
  base_location[1] = -0.02;
  base_location[2] = 0.0;
  tmp_location[0] = base_location[0]*rotation[0][0] + base_location[1]*rotation[0][1] + base_location[2]*rotation[0][2];
  tmp_location[1] = base_location[0]*rotation[1][0] + base_location[1]*rotation[1][1] + base_location[2]*rotation[1][2];
  tmp_location[2] = base_location[0]*rotation[2][0] + base_location[1]*rotation[2][1] + base_location[2]*rotation[2][2];
  base_location[0] = tmp_location[0] + pos[0];
  base_location[1] = tmp_location[1] + pos[1];
  base_location[2] = tmp_location[2] + pos[2];
  right_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(2.0, base_location, orient, line_vel, ang_vel));
  
  base_location[0] = -0.02;
  base_location[1] = 0.02;
  base_location[2] = 0.0;
  tmp_location[0] = base_location[0]*rotation[0][0] + base_location[1]*rotation[0][1] + base_location[2]*rotation[0][2];
  tmp_location[1] = base_location[0]*rotation[1][0] + base_location[1]*rotation[1][1] + base_location[2]*rotation[1][2];
  tmp_location[2] = base_location[0]*rotation[2][0] + base_location[1]*rotation[2][1] + base_location[2]*rotation[2][2];
  base_location[0] = tmp_location[0] + pos[0];
  base_location[1] = tmp_location[1] + pos[1];
  base_location[2] = tmp_location[2] + pos[2];
  right_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(8.0, base_location, orient, line_vel, ang_vel));
  
  base_location[0] = -0.0;
  base_location[1] = 0.02;
  base_location[2] = 0.0;
  tmp_location[0] = base_location[0]*rotation[0][0] + base_location[1]*rotation[0][1] + base_location[2]*rotation[0][2];
  tmp_location[1] = base_location[0]*rotation[1][0] + base_location[1]*rotation[1][1] + base_location[2]*rotation[1][2];
  tmp_location[2] = base_location[0]*rotation[2][0] + base_location[1]*rotation[2][1] + base_location[2]*rotation[2][2];
  base_location[0] = tmp_location[0] + pos[0];
  base_location[1] = tmp_location[1] + pos[1];
  base_location[2] = tmp_location[2] + pos[2];
  right_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(10.0, base_location, orient, line_vel, ang_vel));
  
  base_location[0] = 0.02;
  base_location[1] = 0.02;
  base_location[2] = 0.0;
  tmp_location[0] = base_location[0]*rotation[0][0] + base_location[1]*rotation[0][1] + base_location[2]*rotation[0][2];
  tmp_location[1] = base_location[0]*rotation[1][0] + base_location[1]*rotation[1][1] + base_location[2]*rotation[1][2];
  tmp_location[2] = base_location[0]*rotation[2][0] + base_location[1]*rotation[2][1] + base_location[2]*rotation[2][2];
  base_location[0] = tmp_location[0] + pos[0];
  base_location[1] = tmp_location[1] + pos[1];
  base_location[2] = tmp_location[2] + pos[2];
  right_hand_msg.taskspace_trajectory_points.push_back(trajectory_generation.createSE3(12.0, base_location, orient, line_vel, ang_vel));
  
  ROS_INFO("XYZ location:\t%f\t%f\t%f", plug_point[0] - cable_point[0], plug_point[1] - cable_point[1], plug_point[2] - cable_point[2]);
  
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
  ros::Duration(8.0).sleep();
}

void
Solar_Components_Driver::resetStance(std::vector<float> next_point)
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
  std_msgs::Float64MultiArray left_fingers;
  std_msgs::Float64MultiArray right_fingers;
  std::vector<float> finger_pos(5, 0);
  finger_pos[0] = 1.5;
  finger_pos[1] = 0.0;
  finger_pos[2] = 0.0;
  finger_pos[3] = 0.0;
  finger_pos[4] = 0.0;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
  finger_pos[0] = 1.5;
  finger_pos[1] = -0.0;
  finger_pos[2] = -0.0;
  finger_pos[3] = -0.0;
  finger_pos[4] = -0.0;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
  ros::Duration(1.0).sleep();
  finger_pos[0] = 0.5;
  finger_pos[1] = -0.6;
  finger_pos[2] = -0.2;
  finger_pos[3] = -0.2;
  finger_pos[4] = -0.2;
  left_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryLeftHandPublisher.publish(left_fingers);
  finger_pos[0] = 0.5;
  finger_pos[1] = 0.6;
  finger_pos[2] = 0.2;
  finger_pos[3] = 0.2;
  finger_pos[4] = 0.2;
  right_fingers = trajectory_generation.createFingerTrajectory(finger_pos);
  trajectoryRightHandPublisher.publish(right_fingers);
}

