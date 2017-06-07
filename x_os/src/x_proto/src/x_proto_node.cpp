/*
 * Designed by Tom Xiong
 * 
 * OBJECTIVE:
 * TO TEST THE X_OS ARCHITECTURE
 * 
 * FORMAT:
 * SINGLE COMMAND TEST
 * 
 * OWNERSHIP:
 * TEAM 1476-"XION SYSTEMS"
 * 
 * 
 */

#include "ros/ros.h"
#include "map_scan/map_scan.hpp"
#include "path_generation/path_generation.hpp"
#include "x_publisher/x_publisher.hpp"
#include "gait_generation/gait_generation.hpp"

ros::Publisher scan_output;
pcl::PointCloud<pcl::PointXYZRGB> map;

const bool LEFT = 0;
const bool RIGHT = 1;
const std::string WORLD = "world";
const std::string RIGHT_FOOT_ANKLE = "rightFoot";
const std::string LEFT_FOOT_ANKLE = "leftFoot";
const std::string RIGHT_FOOT_SOLE = "rightCOP_Frame";
const std::string LEFT_FOOT_SOLE = "leftCOP_Frame";
const std::string PELVIS = "pelvis";

Gait_Generation gait_generation;
Trajectory_Generation trajectory_generation;

void
climbStairs(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "x_proto_node");
  ros::NodeHandle mainNodeHandle;
  scan_output = mainNodeHandle.advertise<sensor_msgs::PointCloud2>("/x_os", 1);
  
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Duration(1.0).sleep();
  
  Map_Scan map_scan;
  sensor_msgs::PointCloud2 output;
  Path_Generation path_generation;
  
  map = map_scan.getMap();
  
  //map_scan.wideScan(&map);
  //pcl::toROSMsg(map, output);
  //scan_output.publish(output);
  //map_scan.wideTaskTwoStairScan(&map);
  //pcl::toROSMsg(map, output);
  //scan_output.publish(output);
  //
  //std::vector<float> end_point;
  //path_generation.findTaskThreeStairPoint(&map, &end_point);
  //pcl::toROSMsg(map, output);
  //scan_output.publish(output);
  //path_generation.traversePointList(path_generation.createPathPointList(&tfBuffer, end_point, &map), false);
  
  climbStairs(&tfBuffer, &map);
  
  return 0;
}

void
climbStairs(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  X_Publisher x_publisher;
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
  
  pelvis_msg.execution_mode = ihmc_msgs::PelvisTrajectoryRosMessage::OVERRIDE;
  pelvis_msg.taskspace_trajectory_points.push_back(trajectory_generation.createPelvisHeightTrajectoryPoint(0.5, 1.1));
  
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
  
  //geometry_msgs::TransformStamped left_foot_ankle = tfBuffer->lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  //geometry_msgs::TransformStamped right_foot_ankle = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  //tf2Scalar left_roll;
  //tf2Scalar left_pitch;
  //tf2Scalar left_yaw;
  //tf2Scalar right_roll;
  //tf2Scalar right_pitch;
  //tf2Scalar right_yaw;
  //tf2::Quaternion left_orientation(left_foot_ankle.transform.rotation.x, left_foot_ankle.transform.rotation.y, left_foot_ankle.transform.rotation.z, left_foot_ankle.transform.rotation.w);
  //tf2::Quaternion right_orientation(right_foot_ankle.transform.rotation.x, right_foot_ankle.transform.rotation.y, right_foot_ankle.transform.rotation.z, right_foot_ankle.transform.rotation.w);
  //tf2::Matrix3x3 left_rotation(left_orientation);
  //left_rotation.getRPY(left_roll, left_pitch, left_yaw);
  //tf2::Matrix3x3 right_rotation(right_orientation);
  //right_rotation.getRPY(right_roll, right_pitch, right_yaw);
  //tf2Scalar roll = (left_roll+right_roll)/2;
  //tf2Scalar pitch = (left_pitch+right_pitch)/2;
  //tf2Scalar yaw = (left_yaw+right_yaw)/2;
  //if(left_yaw < -M_PI_2 && right_yaw > M_PI_2)
  //{
  //  if(yaw > 0)
  //  {
  //    yaw -= M_PI;
  //  }
  //  else
  //  {
  //    yaw += M_PI;
  //  }
  //}
  //geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  //center_point.x += right_foot_ankle.transform.translation.x;
  //center_point.y += right_foot_ankle.transform.translation.y;
  //center_point.z += right_foot_ankle.transform.translation.z;
  //
  //center_point.x = center_point.x/2;
  //center_point.y = center_point.y/2;
  //center_point.z = center_point.z/2;
  //ROS_INFO("heading: %f", yaw);
  //
  //int num_angle = round(yaw / M_PI_4);
  //float heading = num_angle * M_PI_4;
  //ROS_INFO("Corrected Heading: %f", heading);
  //
  //std::vector<float> yellow_stair_point(3, 0);
  //
  //// Locate center point between task one handles
  //for(int point_id = 0; point_id < map->points.size(); point_id++)
  //{
  //  if(map->points[point_id].b == 0 && map->points[point_id].g == 255 && map->points[point_id].r == 255)
  //  {
  //    yellow_stair_point[0] = map->points[point_id].x;
  //    yellow_stair_point[1] = map->points[point_id].y;
  //    yellow_stair_point[2] = map->points[point_id].z;
  //  }
  //}
  //float distance_to_stairs = sqrt(pow(fabs(yellow_stair_point[0] - center_point.x), 2.0) + pow(fabs(yellow_stair_point[1] - center_point.y), 2.0));
  
  ros::Duration(0.01).sleep();
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(0.01); // heading - yaw
  x_vector destination;
  //destination.x = distance_to_stairs - 0.42;
  //destination.y = 0;
  //destination.z = 0;
  //step_list_msg = gait_generation.createLowerBodyLinearGaitOffsetQueue(step_list_msg, destination, 0.0);
  //step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  destination.x = 0.27;
  destination.y = 0;
  destination.z = 0.15;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOffsetQueue(step_list_msg, destination, 0.0, 0.26);
  step_list_msg = gait_generation.createLowerBodyEndStepStairQueue(step_list_msg, destination.z);
  destination.x = 2.0;
  destination.y = 0;
  destination.z = 2.0;
  step_list_msg = gait_generation.createLowerBodyLinearGaitOffsetQueue(step_list_msg, destination, 0.0, 0.26);
  step_list_msg = gait_generation.createLowerBodyEndStepStairQueue(step_list_msg, 2.55);
  x_publisher.publish(step_list_msg);
}
