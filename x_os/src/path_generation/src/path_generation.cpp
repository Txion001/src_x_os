#include "path_generation/path_generation.hpp"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"
#include "math.h"
#include "x_publisher/x_publisher.hpp"

Path_Generation::Path_Generation()
{
  ros::NodeHandle trajectory_handler;
  stepListPublisher = trajectory_handler.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list", 1);
  trajectoryListPublisher = trajectory_handler.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/whole_body_trajectory", 1);
}

float
Path_Generation::findMapEndDistance(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  std::vector<pcl::PointXYZRGB> traverse_checkpoint_list;
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
  
  float distance;
  float heading;
  float max_distance;
  // Locate farthest point.
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    distance = sqrt(pow(fabs(map->points[point_id].x-center_point.x), 2.0) + pow(fabs(map->points[point_id].y-center_point.y), 2.0));
    heading = asin((map->points[point_id].y-center_point.y)/distance);
    if(map->points[point_id].x-center_point.x <= 0)
    {
      if(map->points[point_id].y-center_point.y <= 0)
      {
        heading = -M_PI-heading;
      }
      else
      {
        heading = M_PI-heading;
      }
    }
    
    if(max_distance == 0 && heading - yaw > -M_PI/2 && heading - yaw < M_PI/2)
    {
      max_distance = distance;
    }
    else if(distance > max_distance && heading - yaw > -M_PI/2 && heading - yaw < M_PI/2)
    {
      max_distance = distance;
    }
  }
  return max_distance;
}

std::vector<float>
Path_Generation::findMapEndPoint(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  std::vector<pcl::PointXYZRGB> traverse_checkpoint_list;
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
  
  std::vector<float> end_point(3, 0);
  float distance;
  float heading;
  float max_distance;
  // Locate farthest point.
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    distance = sqrt(pow(fabs(map->points[point_id].x-center_point.x), 2.0) + pow(fabs(map->points[point_id].y-center_point.y), 2.0));
    heading = asin((map->points[point_id].y-center_point.y)/distance);
    if(map->points[point_id].x-center_point.x <= 0)
    {
      if(map->points[point_id].y-center_point.y <= 0)
      {
        heading = -M_PI-heading;
      }
      else
      {
        heading = M_PI-heading;
      }
    }
    
    if(max_distance == 0 && heading - yaw > -M_PI/2 && heading - yaw < M_PI/2)
    {
      max_distance = distance;
      end_point[0] = map->points[point_id].x;
      end_point[1] = map->points[point_id].y;
    }
    else if(distance > max_distance && heading - yaw > -M_PI/2 && heading - yaw < M_PI/2)
    {
      max_distance = distance;
      end_point[0] = map->points[point_id].x;
      end_point[1] = map->points[point_id].y;
    }
  }
  std::vector<float> tmp_point(3, 0);
  int num_points = 0;
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    distance = sqrt(pow(fabs(map->points[point_id].x-end_point[0]), 2.0) + pow(fabs(map->points[point_id].y-end_point[1]), 2.0));
    
    if(distance < 1.0)
    {
      tmp_point[0] += map->points[point_id].x;
      tmp_point[1] += map->points[point_id].y;
      num_points++;
    }
  }
  end_point[0] = tmp_point[0]/num_points;
  end_point[1] = tmp_point[1]/num_points;
  
  return end_point;
}

bool
Path_Generation::findMapNextPoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* next_point)
{
  std::vector<pcl::PointXYZRGB> traverse_checkpoint_list;
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
  
  
  ROS_INFO("Current distance: %f", findMapEndDistance(map));
  float distance_to_travel = 4.0;
  if(findMapEndDistance(map) < distance_to_travel)
  {
    if(findMapEndDistance(map) < 2.0)
    {
      return false;
      ROS_INFO("Success: %f", findMapEndDistance(map));
    }
    else
    {
      distance_to_travel = findMapEndDistance(map) - 1.25;
    }
  }
  
  ROS_INFO("Info loaded: %f", yaw);
  int num_checkpoints = 0;
  float distance;
  float heading;
  // Locate a point that's a certain distance from R5's location
  while(num_checkpoints < 3)
  {
    ROS_INFO("New distance check: %f", distance_to_travel);
    num_checkpoints = 0;
    traverse_checkpoint_list.clear();
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      distance = sqrt(pow(fabs(map->points[point_id].x-center_point.x), 2.0) + pow(fabs(map->points[point_id].y-center_point.y), 2.0));
      heading = asin((map->points[point_id].y-center_point.y)/distance);
      if(map->points[point_id].x-center_point.x <= 0)
      {
        if(map->points[point_id].y-center_point.y <= 0)
        {
          heading = -M_PI-heading;
        }
        else
        {
          heading = M_PI-heading;
        }
      }
      if(distance > distance_to_travel - 0.05 && distance < distance_to_travel + 0.05 && heading - yaw/2 > -M_PI/2 && heading - yaw/2 < M_PI/2)
      {
        ROS_INFO("Obtained point: \t%f\t%f", map->points[point_id].x, map->points[point_id].y);
        num_checkpoints++;
        traverse_checkpoint_list.push_back(map->points[point_id]);
      }
    }
    distance_to_travel -= 0.1;
  }
  
  std::vector<float> end_point(3, 0);
  // If there is enough points within that distance range
  // simply take an average of all the points and return that
  // which would in theory provide a point in the middle of the path.
  for(int point_id = 1; point_id < traverse_checkpoint_list.size(); point_id++)
  {
    ROS_INFO("Get average");
    traverse_checkpoint_list[0].x += traverse_checkpoint_list[point_id].x;
    traverse_checkpoint_list[0].y += traverse_checkpoint_list[point_id].y;
    traverse_checkpoint_list[0].z += traverse_checkpoint_list[point_id].z;
  }
  end_point[0] = traverse_checkpoint_list[0].x/traverse_checkpoint_list.size();
  end_point[1] = traverse_checkpoint_list[0].y/traverse_checkpoint_list.size();
  end_point[2] = traverse_checkpoint_list[0].z/traverse_checkpoint_list.size();
  if(traverse_checkpoint_list.size() < 10)
  {
    traverse_checkpoint_list.clear();
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      distance = sqrt(pow(fabs(map->points[point_id].x-end_point[0]), 2.0) + pow(fabs(map->points[point_id].y-end_point[1]), 2.0));
      if(distance < 1.50)
      {
	      traverse_checkpoint_list.push_back(map->points[point_id]);
      }
    }
    for(int point_id = 1; point_id < traverse_checkpoint_list.size(); point_id++)
    {
      traverse_checkpoint_list[0].x += traverse_checkpoint_list[point_id].x;
      traverse_checkpoint_list[0].y += traverse_checkpoint_list[point_id].y;
      traverse_checkpoint_list[0].z += traverse_checkpoint_list[point_id].z;
    }
    end_point[0] = traverse_checkpoint_list[0].x/traverse_checkpoint_list.size();
    end_point[1] = traverse_checkpoint_list[0].y/traverse_checkpoint_list.size();
    end_point[2] = traverse_checkpoint_list[0].z/traverse_checkpoint_list.size();
  }
  ROS_INFO("New checkpoint found XYZ:\t%f\t%f\t%f", end_point[0], end_point[1], end_point[2]);
  *next_point = end_point;
  
  if(findMapEndDistance(map) < 4.0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

std::vector<float>
Path_Generation::findMapNextPoint(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  std::vector<pcl::PointXYZRGB> traverse_checkpoint_list;
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
  
  ROS_INFO("Info loaded: %f", yaw);
  int num_checkpoints = 0;
  float distance_to_travel = 4.0;
  float distance;
  float heading;
  // Locate a point that's a certain distance from R5's location
  while(num_checkpoints < 3)
  {
    num_checkpoints = 0;
    traverse_checkpoint_list.clear();
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      distance = sqrt(pow(fabs(map->points[point_id].x-center_point.x), 2.0) + pow(fabs(map->points[point_id].y-center_point.y), 2.0));
      heading = asin((map->points[point_id].y-center_point.y)/distance);
      if(map->points[point_id].x-center_point.x <= 0)
      {
        if(map->points[point_id].y-center_point.y <= 0)
        {
          heading = -M_PI-heading;
        }
        else
        {
          heading = M_PI-heading;
        }
      }
      if(distance > distance_to_travel - 0.05 && distance < distance_to_travel + 0.05 && heading - yaw/2 > -M_PI/3 && heading - yaw/2 < M_PI/3)
      {
        ROS_INFO("Obtained point: \t%f\t%f", map->points[point_id].x, map->points[point_id].y);
        num_checkpoints++;
        traverse_checkpoint_list.push_back(map->points[point_id]);
      }
    }
    distance_to_travel -= 0.1;
  }
  
  std::vector<float> end_point(3, 0);
  // If there is enough points within that distance range
  // simply take an average of all the points and return that
  // which would in theory provide a point in the middle of the path.
  for(int point_id = 1; point_id < traverse_checkpoint_list.size(); point_id++)
  {
    ROS_INFO("Get average");
    traverse_checkpoint_list[0].x += traverse_checkpoint_list[point_id].x;
    traverse_checkpoint_list[0].y += traverse_checkpoint_list[point_id].y;
    traverse_checkpoint_list[0].z += traverse_checkpoint_list[point_id].z;
  }
  end_point[0] = traverse_checkpoint_list[0].x/traverse_checkpoint_list.size();
  end_point[1] = traverse_checkpoint_list[0].y/traverse_checkpoint_list.size();
  end_point[2] = traverse_checkpoint_list[0].z/traverse_checkpoint_list.size();
  if(traverse_checkpoint_list.size() < 10)
  {
    traverse_checkpoint_list.clear();
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      distance = sqrt(pow(fabs(map->points[point_id].x-end_point[0]), 2.0) + pow(fabs(map->points[point_id].y-end_point[1]), 2.0));
      if(distance < 1.50)
      {
	      traverse_checkpoint_list.push_back(map->points[point_id]);
      }
    }
    for(int point_id = 1; point_id < traverse_checkpoint_list.size(); point_id++)
    {
      traverse_checkpoint_list[0].x += traverse_checkpoint_list[point_id].x;
      traverse_checkpoint_list[0].y += traverse_checkpoint_list[point_id].y;
      traverse_checkpoint_list[0].z += traverse_checkpoint_list[point_id].z;
    }
    end_point[0] = traverse_checkpoint_list[0].x/traverse_checkpoint_list.size();
    end_point[1] = traverse_checkpoint_list[0].y/traverse_checkpoint_list.size();
    end_point[2] = traverse_checkpoint_list[0].z/traverse_checkpoint_list.size();
  }
  ROS_INFO("New checkpoint found XYZ:\t%f\t%f\t%f", end_point[0], end_point[1], end_point[2]);
  return end_point;
}

std::vector<float>
Path_Generation::findMapNextPoint(pcl::PointCloud<pcl::PointXYZRGB>* map, float distance_to_travel)
{
  std::vector<pcl::PointXYZRGB> traverse_checkpoint_list;
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
  
  
  ROS_INFO("Info loaded: %f", yaw);
  int num_checkpoints = 0;
  float distance;
  float heading;
  // Locate a point that's a certain distance from R5's location
  while(num_checkpoints < 3)
  {
    num_checkpoints = 0;
    traverse_checkpoint_list.clear();
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      distance = sqrt(pow(fabs(map->points[point_id].x-center_point.x), 2.0) + pow(fabs(map->points[point_id].y-center_point.y), 2.0));
      heading = asin((map->points[point_id].y-center_point.y)/distance);
      if(map->points[point_id].x-center_point.x <= 0)
      {
        if(map->points[point_id].y-center_point.y <= 0)
        {
          heading = -M_PI-heading;
        }
        else
        {
          heading = M_PI-heading;
        }
      }
      if(distance > distance_to_travel - 0.08 && distance < distance_to_travel + 0.08 && heading - yaw/2 > -M_PI/2 && heading - yaw/2 < M_PI/2)
      {
        //ROS_INFO("Obtained point: \t%f\t%f", map->points[point_id].x, map->points[point_id].y);
        num_checkpoints++;
        traverse_checkpoint_list.push_back(map->points[point_id]);
      }
    }
    distance_to_travel -= 0.1;
  }
  
  std::vector<float> end_point(4, 0);
  std::vector<float> tmp_point(3, 0);
  tmp_point = findMapEndPoint(map);
  ROS_INFO("New checkpoint found XYZ:\t%f\t%f\t%f", tmp_point[0], tmp_point[1], tmp_point[2]);
  float least_distance = 0;
  for(int point_id = 0; point_id < traverse_checkpoint_list.size(); point_id++)
  {
    distance = sqrt(pow(fabs(traverse_checkpoint_list[point_id].x-tmp_point[0]), 2.0) + pow(fabs(traverse_checkpoint_list[point_id].y-tmp_point[1]), 2.0));
    if(point_id == 0)
    {
      least_distance = distance;
      end_point[0] = traverse_checkpoint_list[point_id].x;
      end_point[1] = traverse_checkpoint_list[point_id].y;
    }
    else if(distance < least_distance)
    {
      least_distance = distance;
      end_point[0] = traverse_checkpoint_list[point_id].x;
      end_point[1] = traverse_checkpoint_list[point_id].y;
    }
  }
  tmp_point[0] = 0;
  tmp_point[1] = 0;
  num_checkpoints = 0;
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    distance = sqrt(pow(fabs(map->points[point_id].x-end_point[0]), 2.0) + pow(fabs(map->points[point_id].y-end_point[1]), 2.0));
    if(distance < 2.0)
    {
      //ROS_INFO("Obtained point: \t%f\t%f", map->points[point_id].x, map->points[point_id].y);
      num_checkpoints++;
      tmp_point[0] += map->points[point_id].x;
      tmp_point[1] += map->points[point_id].y;
    }
  }
  end_point[0] = tmp_point[0]/num_checkpoints;
  end_point[1] = tmp_point[1]/num_checkpoints;
  ROS_INFO("New checkpoint found XYZ:\t%f\t%f\t%f", end_point[0], end_point[1], end_point[2]);
  
  tmp_point = findMapEndPoint(map);
  float end_final_distance = sqrt(pow(fabs(tmp_point[0] - end_point[0]), 2.0) + pow(fabs(tmp_point[1] - end_point[1]), 2.0));
  heading = asin((tmp_point[1] - end_point[1])/end_final_distance);
  end_point[3] = heading;
  
  return end_point;
}

bool
Path_Generation::findTaskOnePoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output)
{
  std::vector<float> red_handle_point(3, 0);
  std::vector<float> blue_handle_point(3, 0);
  std::vector<float> task_point(4, 0);
  float heading = 0;
  bool obtained_red_handle = false;
  bool obtained_blue_handle = false;
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
    {
      red_handle_point[0] = map->points[point_id].x;
      red_handle_point[1] = map->points[point_id].y;
      red_handle_point[2] = map->points[point_id].z;
      obtained_red_handle = true;
    }
    else if(map->points[point_id].g == 0 && map->points[point_id].r == 0 && map->points[point_id].b == 200)
    {
      blue_handle_point[0] = map->points[point_id].x;
      blue_handle_point[1] = map->points[point_id].y;
      blue_handle_point[2] = map->points[point_id].z;
      obtained_blue_handle = true;
    }
  }
  if(obtained_red_handle == false || obtained_blue_handle == false)
  {
    return false;
  }
  
  task_point[0] = (red_handle_point[0] + blue_handle_point[0]) / 2;
  task_point[1] = (red_handle_point[1] + blue_handle_point[1]) / 2;
  task_point[2] = (red_handle_point[2] + blue_handle_point[2]) / 2;
  
  float handle_seperation = sqrt(pow(fabs(red_handle_point[0] - blue_handle_point[0]), 2.0) + pow(fabs(red_handle_point[1] - blue_handle_point[1]), 2.0));
  heading = asin((red_handle_point[1] - blue_handle_point[1])/handle_seperation);
  if(red_handle_point[0] - blue_handle_point[0] <= 0)
  {
    if(red_handle_point[1] - blue_handle_point[1] <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  heading -= M_PI/2;
  
  std::vector<float> distance_from_table_point(3, 0);
  distance_from_table_point[0] = -0.65;
  tf2::Matrix3x3 table_direction;
  table_direction.setRPY(0.0, 0.0, heading);
  
  
  // Convert this distance into a location away from front of table
  task_point[0] += distance_from_table_point[0]*table_direction[0][0] + distance_from_table_point[1]*table_direction[0][1] + distance_from_table_point[2]*table_direction[0][2];
  task_point[1] += distance_from_table_point[0]*table_direction[1][0] + distance_from_table_point[1]*table_direction[1][1] + distance_from_table_point[2]*table_direction[1][2];
  task_point[3] = heading;

  ROS_INFO("XYYAW location:\t%f\t%f\t%f", task_point[0], task_point[1], heading);
  
  *output = task_point;
  return true;
}

bool
Path_Generation::findTaskTwoPanelPoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output)
{
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
  std::vector<float> handle_point(3, 0);
  std::vector<float> trailer_point(3, 0);
  std::vector<float> task_point(4, 0);
  task_point[0] = 0;
  task_point[1] = 0;
  task_point[2] = 0;
  float heading = 0;
  bool found_panel = false;
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 200 && map->points[point_id].g == 0 && map->points[point_id].r == 0)
    {
      found_panel = true;
      handle_point[0] = map->points[point_id].x;
      handle_point[1] = map->points[point_id].y;
      handle_point[2] = map->points[point_id].z;
    }
    else if(map->points[point_id].b == 10 && map->points[point_id].g == 10 && map->points[point_id].r == 10)
    {
      found_panel = true;
      trailer_point[0] = map->points[point_id].x;
      trailer_point[1] = map->points[point_id].y;
      trailer_point[2] = map->points[point_id].z;
    }
  }
  ROS_INFO("XYZ location:\t%f\t%f\t%f", handle_point[0], handle_point[1], handle_point[2]);
  ROS_INFO("XYZ location:\t%f\t%f\t%f", trailer_point[0], trailer_point[1], trailer_point[2]);
  
  float handle_seperation = sqrt(pow(fabs(trailer_point[0] - handle_point[0]), 2.0) + pow(fabs(trailer_point[1] - handle_point[1]), 2.0));
  heading = asin((trailer_point[1] - handle_point[1])/handle_seperation);
  if(trailer_point[0] - handle_point[0] <= 0)
  {
    if(trailer_point[1] - handle_point[1] <= 0)
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
  std::vector<float> end_point(3, 0);
  end_point[0] = -0.9;
  end_point[1] = 0.0;
  end_point[2] = 0.0;
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, heading);
  
  task_point[0] = end_point[0]*rotation[0][0] + end_point[1]*rotation[0][1] + end_point[2]*rotation[0][2] + handle_point[0];
  task_point[1] = end_point[0]*rotation[1][0] + end_point[1]*rotation[1][1] + end_point[2]*rotation[1][2] + handle_point[1];
  task_point[2] = end_point[0]*rotation[2][0] + end_point[1]*rotation[2][1] + end_point[2]*rotation[2][2] + handle_point[2];
  task_point[3] = heading;
  
  ROS_INFO("XYYAW location:\t%f\t%f\t%f", task_point[0], task_point[1], task_point[3]);
  
  float handle_final_distance = sqrt(pow(fabs(handle_point[0] - center_point.x), 2.0) + pow(fabs(handle_point[1] - center_point.y), 2.0));
  ROS_INFO("Final Distance: %f", handle_final_distance);
  *output = task_point;
  if(found_panel == true && handle_final_distance < 3.0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool
Path_Generation::findTaskTwoArrayPoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output)
{
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
  std::vector<float> blue_cable_point(3, 0);
  std::vector<float> red_wire_point(3, 0);
  bool found_cable = false;
  bool found_wire = false;
  float heading = 0;
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200 && !isnan(map->points[point_id].x))
    {
      found_wire = true;
      red_wire_point[0] = map->points[point_id].x;
      red_wire_point[1] = map->points[point_id].y;
      red_wire_point[2] = map->points[point_id].z;
    }
    else if(map->points[point_id].g == 0 && map->points[point_id].r == 0 && map->points[point_id].b == 200)
    {
      found_cable = true;
      blue_cable_point[0] = map->points[point_id].x;
      blue_cable_point[1] = map->points[point_id].y;
      blue_cable_point[2] = map->points[point_id].z;
    }
  }
  
  float cable_final_distance = sqrt(pow(fabs(red_wire_point[0] - blue_cable_point[0]), 2.0) + pow(fabs(red_wire_point[1] - blue_cable_point[1]), 2.0));
  heading = asin((red_wire_point[1] - blue_cable_point[1])/cable_final_distance);
  ROS_INFO("XYZ location:\t%f\t%f\t%f\tHeading: %f", red_wire_point[0], red_wire_point[1], red_wire_point[2], heading);
  if(red_wire_point[0] - blue_cable_point[0] <= 0)
  {
    if(red_wire_point[1] - blue_cable_point[1] <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  ROS_INFO("XYZ location:\t%f\t%f\t%f\tHeading: %f", blue_cable_point[0], blue_cable_point[1], blue_cable_point[2], heading);
  
  int num_angle = round(heading / M_PI_4);
  heading = num_angle * M_PI_4;
  ROS_INFO("Heading: %f", heading);
  
  std::vector<float> end_point(3, 0);
  end_point[0] = -0.9;
  end_point[1] = 0.4;
  end_point[2] = 0.0;
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, heading);
  
  std::vector<float> task_point(4, 0);
  task_point[0] = end_point[0]*rotation[0][0] + end_point[1]*rotation[0][1] + end_point[2]*rotation[0][2];
  task_point[1] = end_point[0]*rotation[1][0] + end_point[1]*rotation[1][1] + end_point[2]*rotation[1][2];
  task_point[2] = end_point[0]*rotation[2][0] + end_point[1]*rotation[2][1] + end_point[2]*rotation[2][2];
  task_point[0] += blue_cable_point[0];
  task_point[1] += blue_cable_point[1];
  task_point[2] = 0;
  task_point[3] = heading;
  
  ROS_INFO("XYYAW location:\t%f\t%f\t%f", task_point[0], task_point[1], task_point[3]);
  
  *output = task_point;
  if(found_cable == true && found_wire && !isnan(task_point[0]) && !isnan(task_point[1]))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool
Path_Generation::findTaskTwoArrayPointClose(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output)
{
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
  std::vector<float> blue_cable_point(3, 0);
  std::vector<float> red_wire_point(3, 0);
  bool found_cable = false;
  bool found_wire = false;
  float heading = 0;
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
    {
      found_wire = true;
      red_wire_point[0] = map->points[point_id].x;
      red_wire_point[1] = map->points[point_id].y;
      red_wire_point[2] = map->points[point_id].z;
    }
    else if(map->points[point_id].g == 0 && map->points[point_id].r == 0 && map->points[point_id].b == 200)
    {
      found_cable = true;
      blue_cable_point[0] = map->points[point_id].x;
      blue_cable_point[1] = map->points[point_id].y;
      blue_cable_point[2] = map->points[point_id].z;
    }
  }
  
  float cable_final_distance = sqrt(pow(fabs(red_wire_point[0] - blue_cable_point[0]), 2.0) + pow(fabs(red_wire_point[1] - blue_cable_point[1]), 2.0));
  heading = asin((red_wire_point[1] - blue_cable_point[1])/cable_final_distance);
  ROS_INFO("XYZ location:\t%f\t%f\t%f\tHeading: %f", red_wire_point[0], red_wire_point[1], red_wire_point[2], heading);
  if(red_wire_point[0] - blue_cable_point[0] <= 0)
  {
    if(red_wire_point[1] - blue_cable_point[1] <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  ROS_INFO("XYZ location:\t%f\t%f\t%f\tHeading: %f", blue_cable_point[0], blue_cable_point[1], blue_cable_point[2], heading);
  
  int num_angle = round(heading / M_PI_4);
  heading = num_angle * M_PI_4;
  ROS_INFO("Heading: %f", heading);
  if(heading < -M_PI || heading > M_PI)
  {
    if(heading > 0)
    {
      heading -= 2*M_PI;
    }
    else
    {
      heading += 2*M_PI;
    }
  }
  
  std::vector<float> end_point(3, 0);
  end_point[0] = -0.6;
  end_point[1] = 0.4;
  end_point[2] = 0.0;
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, heading);
  
  std::vector<float> task_point(4, 0);
  task_point[0] = end_point[0]*rotation[0][0] + end_point[1]*rotation[0][1] + end_point[2]*rotation[0][2] + blue_cable_point[0];
  task_point[1] = end_point[0]*rotation[1][0] + end_point[1]*rotation[1][1] + end_point[2]*rotation[1][2] + blue_cable_point[1];
  task_point[2] = end_point[0]*rotation[2][0] + end_point[1]*rotation[2][1] + end_point[2]*rotation[2][2];
  task_point[3] = heading;
  
  ROS_INFO("XYYAW End location:\t%f\t%f\t%f", task_point[0], task_point[1], task_point[3]);
  
  *output = task_point;
  if(found_cable == true && found_wire && !isnan(task_point[0]) && !isnan(task_point[1]))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool
Path_Generation::findTaskThreeStairPoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output)
{
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
  std::vector<float> yellow_stair_point(3, 0);
  std::vector<float> task_point(4, 0);
  bool found_stairs = false;
  float heading = 0;
  
  // Locate center point between task one handles
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 255 && map->points[point_id].r == 255)
    {
      found_stairs = true;
      yellow_stair_point[0] = map->points[point_id].x;
      yellow_stair_point[1] = map->points[point_id].y;
      yellow_stair_point[2] = map->points[point_id].z;
    }
  }
  ROS_INFO("XYZ location:\t%f\t%f\t%f", yellow_stair_point[0], yellow_stair_point[1], yellow_stair_point[2]);
  int num_adds = 0;
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    float near_zone = sqrt(pow(fabs(yellow_stair_point[0] - map->points[point_id].x), 2.0) + pow(fabs(yellow_stair_point[1] - map->points[point_id].y), 2.0));
    if(near_zone < 1.0 && (map->points[point_id].r == map->points[point_id].g && map->points[point_id].g == map->points[point_id].b && map->points[point_id].b < 50))
    {
      task_point[0] += map->points[point_id].x;
      task_point[1] += map->points[point_id].y;
      task_point[2] += map->points[point_id].z;
      num_adds++;
    }
  }
  task_point[0] = task_point[0] / num_adds;
  task_point[1] = task_point[1] / num_adds;
  task_point[2] = task_point[2] / num_adds;
  
  float stair_final_distance = sqrt(pow(fabs(yellow_stair_point[0] - task_point[0]), 2.0) + pow(fabs(yellow_stair_point[1] - task_point[1]), 2.0));
  heading = asin((yellow_stair_point[1] - task_point[1])/stair_final_distance);
  if(yellow_stair_point[0] - task_point[0] <= 0)
  {
    if(yellow_stair_point[1] - task_point[1] <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  ROS_INFO("XYZ location:\t%f\t%f\t%f\tHeading: %f", task_point[0], task_point[1], task_point[2], heading);
  
  int num_angle = round(heading / M_PI_4);
  heading = num_angle * M_PI_4;
  ROS_INFO("Heading: %f", heading);
  
  std::vector<float> end_point(3, 0);
  end_point[0] = -1.0;
  end_point[1] = 0.0;
  end_point[2] = 0.0;
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, heading);
  
  task_point[0] = end_point[0]*rotation[0][0] + end_point[1]*rotation[0][1] + end_point[2]*rotation[0][2];
  task_point[1] = end_point[0]*rotation[1][0] + end_point[1]*rotation[1][1] + end_point[2]*rotation[1][2];
  task_point[2] = end_point[0]*rotation[2][0] + end_point[1]*rotation[2][1] + end_point[2]*rotation[2][2];
  task_point[0] += yellow_stair_point[0];
  task_point[1] += yellow_stair_point[1];
  task_point[2] = 0;
  task_point[3] = heading;
  
  stair_final_distance = sqrt(pow(fabs(center_point.x - task_point[0]), 2.0) + pow(fabs(center_point.y - task_point[1]), 2.0));
  
  *output = task_point;
  if(found_stairs == true && !isnan(task_point[0]) && !isnan(task_point[1]) && stair_final_distance < 3.0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*
 * Designed to find obstacles between current robot location
 * and endpoint. Then utilize obstacle avoidence to produce
 * new check points for valkyrie to walk to.
 * 
 */
std::vector<std::vector<float>>
Path_Generation::createPathPointList(tf2_ros::Buffer* tfBuffer, std::vector<float> end_point, pcl::PointCloud<pcl::PointXYZRGB>* map)/// TODO: Create a full 360 degree path planner that excludes return 
{
  //ROS_INFO("Section Start.");
  ros::NodeHandle mainNodeHandle;
  std::vector<std::vector<float>> path_point_list;
  
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
  
  float current_x_checkpoint = center_point.x;
  float current_y_checkpoint = center_point.y;
  const int obstacle_count = 21;
  const float radius_of_zone_to_check = 0.42;
  
  ROS_INFO("Current foot position: %f\t%f", current_x_checkpoint, current_y_checkpoint);
  
  bool cleared_obstacles = false;
  bool reached_closing_distance = false;
  //while(cleared_obstacles == false && mainNodeHandle.ok())
  //{
  for(int num_obstacle_avoidances = 0; num_obstacle_avoidances < 2 && cleared_obstacles == false && reached_closing_distance == false; num_obstacle_avoidances++)
  {
    ROS_INFO("Section determine whether path has obstacle");
    // Determine if the current path has an obstacle.
    // Calculate points along path to end location
    float x_end_point = end_point[0] - current_x_checkpoint;
    float y_end_point = end_point[1] - current_y_checkpoint;
    float cross_distance = sqrt(pow(x_end_point, 2.0) + pow(y_end_point, 2.0));
    float num_checks = cross_distance/0.1;
    float path_increment = cross_distance/num_checks;
    float max_single_traverse_distance = 2.5;
    
    // Check along path for an obstacle.
    bool path_has_obstacles = false;
    for(float current_check_increment = 0.0; current_check_increment < cross_distance - 0.25 && path_has_obstacles == false; current_check_increment += path_increment)
    {
      int path_point_has_obstacles = obstacle_count;
      float current_x_pathpoint = x_end_point * (current_check_increment/cross_distance) + current_x_checkpoint;
      float current_y_pathpoint = y_end_point * (current_check_increment/cross_distance) + current_y_checkpoint;
      //ROS_INFO("Checking xy points: \t%f\t%f", current_x_pathpoint, current_y_pathpoint);
      
      for(int point_id = 0; point_id < map->points.size() && path_point_has_obstacles > 0; point_id++)
      {
        float distance_from_point = sqrt(pow(map->points[point_id].x - current_x_pathpoint, 2.0) + pow(map->points[point_id].y - current_y_pathpoint, 2.0));
        if(distance_from_point < radius_of_zone_to_check && map->points[point_id].r != 255 && map->points[point_id].r != 200 && map->points[point_id].r != 180)
        {
          path_point_has_obstacles -= 1;
        }
      }
      //pcl::PointXYZRGB temp_point;
      //temp_point.x = current_x_pathpoint;
      //temp_point.y = current_y_pathpoint;
      //temp_point.z = 0.0;
      //temp_point.r = 200;
      //temp_point.g = 200;
      //temp_point.b = 0;
      //map->points.push_back(temp_point);
      if(path_point_has_obstacles > 0)
      {
        //ROS_INFO("Obstacle Found\n");
        path_has_obstacles = true;
      }
    }
    
    // If path has an obstacle, perform a path sweep
    // check to look for a new location which would
    // produce a point closer to the goal without in
    // between obstacles.
    struct new_checkpoint
    {
      float x_position;
      float y_position;
      float heading;
    }checkpoint;
    checkpoint.x_position = 0;
    checkpoint.y_position = 0;
    std::vector<new_checkpoint> checkpoint_list;
    if(path_has_obstacles == true)
    {
      //ROS_INFO("Section find new path");
      tf2::Matrix3x3 possible_path_heading;
      float deg_increment = 10;
      float radian_increment = deg_increment/360*M_PI;
      // Scan for nth number of possible paths in a
      // right to left sweep with a specific radian
      // increment from -45 deg to 45 deg.
      for(float current_radian_increment = -M_PI/2 - yaw/2; current_radian_increment < M_PI/2 - yaw/2; current_radian_increment += radian_increment)
      {
        possible_path_heading.setRPY(0.0, 0.0, current_radian_increment);
        
        float possible_forward_pos = 0.30;
        std::vector<float> axis_comparison(2, 0);
        bool has_reached_max_distance = false;
        
        float closing_distance = sqrt(pow(fabs(current_x_checkpoint - end_point[0]), 2.0) + pow(fabs(current_y_checkpoint - end_point[1]), 2.0));
        int possible_path_encountered_obstacle = 0;
        // Increase distance of point to check in new heading
        new_checkpoint temp_checkpoint;
        temp_checkpoint.x_position = current_x_checkpoint;
        temp_checkpoint.y_position = current_y_checkpoint;
        while(possible_path_encountered_obstacle <= 0 && closing_distance > 0.5 && mainNodeHandle.ok())
        {
          int point_id =0;
          checkpoint.x_position = current_x_checkpoint;
          checkpoint.y_position = current_y_checkpoint;
          possible_path_encountered_obstacle = obstacle_count;
          axis_comparison[0] = possible_forward_pos*possible_path_heading[0][0] + current_x_checkpoint;
          axis_comparison[1] = possible_forward_pos*possible_path_heading[1][0] + current_y_checkpoint;
          
          // Iterate through map to find whether point exists
          for(point_id = 0; point_id < map->points.size(); point_id++)
          {
            float distance_from_point = sqrt(pow(map->points[point_id].x - axis_comparison[0], 2.0) + pow(map->points[point_id].y - axis_comparison[1], 2.0));
            if(distance_from_point < radius_of_zone_to_check && map->points[point_id].r != 255 && map->points[point_id].r != 200 && map->points[point_id].r != 180)
            {
              possible_path_encountered_obstacle -= 1;
            }
          }
          
          closing_distance = sqrt(pow(fabs(axis_comparison[0] - end_point[0]), 2.0) + pow(fabs(axis_comparison[1] - end_point[1]), 2.0));
          if(closing_distance <= 0.5)
          {
			reached_closing_distance = true;
            ROS_INFO("Reached within 0.5m from destination: %f\t%f", axis_comparison[0], axis_comparison[1]);
            checkpoint_list.push_back(temp_checkpoint);
          }
          if(possible_forward_pos > max_single_traverse_distance)
          {
            ROS_INFO("Reached max distance test: %f\t%f", axis_comparison[0], axis_comparison[1]);
            has_reached_max_distance = true;
          }
          if(possible_path_encountered_obstacle > 0)
          {
            ROS_INFO("Completed single sweep scan: %f", possible_forward_pos);
            if(static_cast<int>(temp_checkpoint.x_position) != 0)
            {
              if(temp_checkpoint.x_position == 0)
              {
                checkpoint.x_position = current_x_checkpoint;
                checkpoint.y_position = current_y_checkpoint;
                temp_checkpoint = checkpoint;
              }
              if(has_reached_max_distance == true && possible_forward_pos > max_single_traverse_distance + 1.0)
              {
                possible_forward_pos = max_single_traverse_distance;
                temp_checkpoint.x_position = possible_forward_pos*possible_path_heading[0][0] + current_x_checkpoint;
                temp_checkpoint.y_position = possible_forward_pos*possible_path_heading[1][0] + current_y_checkpoint;
                ROS_INFO("Reached max distance: %f\t%f", temp_checkpoint.x_position, temp_checkpoint.y_position);
                checkpoint_list.push_back(temp_checkpoint);
                //pcl::PointXYZRGB temp_point;
                //temp_point.x = temp_checkpoint.x_position;
                //temp_point.y = temp_checkpoint.y_position;
                //temp_point.z = 0.0;
                //temp_point.r = 0;
                //temp_point.g = 0;
                //temp_point.b = 180;
                //map->points.push_back(temp_point);
              }
              else
              {
                possible_forward_pos -= 0.75;
                temp_checkpoint.x_position = possible_forward_pos*possible_path_heading[0][0] + current_x_checkpoint;
                temp_checkpoint.y_position = possible_forward_pos*possible_path_heading[1][0] + current_y_checkpoint;
                ROS_INFO("Reached obstacle: %f\t%f", temp_checkpoint.x_position, temp_checkpoint.y_position);
                checkpoint_list.push_back(temp_checkpoint);
                pcl::PointXYZRGB temp_point;
                //temp_point.x = temp_checkpoint.x_position;
                //temp_point.y = temp_checkpoint.y_position;
                //temp_point.z = 0.0;
                //temp_point.r = 180;
                //temp_point.g = 0;
                //temp_point.b = 0;
                //map->points.push_back(temp_point);
              }
            }
          }
          else
          {
            checkpoint.x_position = axis_comparison[0];
            checkpoint.y_position = axis_comparison[1];
            if(static_cast<int>(checkpoint.x_position) != 0)
            {
              temp_checkpoint = checkpoint;
            }
            //pcl::PointXYZRGB temp_point;
            //temp_point.x = checkpoint.x_position;
            //temp_point.y = checkpoint.y_position;
            //temp_point.z = 0.0;
            //temp_point.r = 255;
            //temp_point.g = 255;
            //temp_point.b = 255;
            //map->points.push_back(temp_point);
          }
          possible_forward_pos += 0.1;
        }
      }
      float closest_to_end_point = -1;
      float current_to_end_point = 0;
      for(int checkpoint_id = 0; checkpoint_id < checkpoint_list.size(); checkpoint_id++)
      {
        current_to_end_point = sqrt(pow(end_point[0] - checkpoint_list[checkpoint_id].x_position, 2.0) + pow(end_point[1] - checkpoint_list[checkpoint_id].y_position, 2.0));
        ROS_INFO("Distance compare: \t%f\t%f", current_to_end_point, closest_to_end_point);
        if(closest_to_end_point < 0)
        {
          closest_to_end_point = current_to_end_point;
          checkpoint.x_position = checkpoint_list[checkpoint_id].x_position;
          checkpoint.y_position = checkpoint_list[checkpoint_id].y_position;
        }
        else if(current_to_end_point < closest_to_end_point)
        {
          closest_to_end_point = current_to_end_point;
          checkpoint.x_position = checkpoint_list[checkpoint_id].x_position;
          checkpoint.y_position = checkpoint_list[checkpoint_id].y_position;
          ROS_INFO("Target point: \t%f\t%f", end_point[0], end_point[1]);
          ROS_INFO("Next close xy checkpoint found: \t%f\t%f", checkpoint.x_position, checkpoint.y_position);
        }
      }
      ROS_INFO("\nClosest xy checkpoint found: \t%f\t%f", checkpoint.x_position, checkpoint.y_position);
      //pcl::PointXYZRGB temp_point;
      //temp_point.x = checkpoint.x_position;
      //temp_point.y = checkpoint.y_position;
      //temp_point.z = 0.0;
      //temp_point.r = 200;
      //temp_point.g = 200;
      //temp_point.b = 0;
      //map->points.push_back(temp_point);
      
      // Take new_path checkpoint and add it to the checkpoint
      // list. Then set the new checkpoint origin to the
      // just newly created checkpoint
      current_x_checkpoint = checkpoint.x_position;
      current_y_checkpoint = checkpoint.y_position;
      std::vector<float> temp_checkpoint(3, 0);
      temp_checkpoint[0] = checkpoint.x_position;
      temp_checkpoint[1] = checkpoint.y_position;
      path_point_list.push_back(temp_checkpoint);
    }
    // If the current path doesn't have an obstacle
    // just add the goal point to the path list and
    // declare the new path check points clears all
    // obstacles
    else
    {
      ROS_INFO("Section all obstacles dodged: \t%f\t%f", end_point[0], end_point[1]);
      cleared_obstacles = true;
    }
  }
  path_point_list.push_back(end_point);
  //pcl::PointXYZRGB temp_point;
  //temp_point.x = end_point[0];
  //temp_point.y = end_point[1];
  //temp_point.z = 0.0;
  //temp_point.r = 0;
  //temp_point.g = 200;
  //temp_point.b = 0;
  //map->points.push_back(temp_point);
  
  return path_point_list;
}

void
Path_Generation::traverseToPoint(std::vector<float> end_point, bool arms_disabled)
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
  ROS_INFO("Starting heading: %f", yaw);
  
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  float heading = 0;
  float total_heading = yaw;
  float x_distance_to_travel = end_point[0] - center_point.x;
  float y_distance_to_travel = end_point[1] - center_point.y;
  float z_distance_to_travel = end_point[2] - center_point.z;
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, -total_heading);
  
  // Check if the points are behind valkyrie, thus prompting a rotation offset
  std::vector<float> point_comparison(3, 0);
  point_comparison[0] = x_distance_to_travel*rotation[0][0] + y_distance_to_travel*rotation[0][1] + z_distance_to_travel*rotation[0][2];
  point_comparison[1] = x_distance_to_travel*rotation[1][0] + y_distance_to_travel*rotation[1][1] + z_distance_to_travel*rotation[1][2];
  
  float distance = sqrt(pow(point_comparison[0], 2.0) + pow(point_comparison[1], 2.0));
  float new_heading = asin(point_comparison[1]/distance);
  ROS_INFO("XYOffset: %f\t%f", point_comparison[0], point_comparison[1]);
  if(point_comparison[0] <= 0)
  {
    if(point_comparison[1] <= 0)
    {
      total_heading += -M_PI-new_heading;
      heading = -M_PI-new_heading;
    }
    else
    {
      total_heading += M_PI-new_heading;
      heading = M_PI-new_heading;
    }
  }
  else
  {
    heading = new_heading;
    total_heading += new_heading;
  }
  if(heading < -M_PI || heading > M_PI)
  {
    if(heading > 0)
    {
      heading -= 2*M_PI;
    }
    else
    {
      heading += 2*M_PI;
    }
  }
  if(total_heading < -M_PI || total_heading > M_PI)
  {
    if(total_heading > 0)
    {
      total_heading -= 2*M_PI;
    }
    else
    {
      total_heading += 2*M_PI;
    }
  }
  ROS_INFO("Distance: %f\tHeading: %f\tRaw Heading: %f\tTotal Heading: %f", distance, heading, new_heading, total_heading);
  
  if(heading > 3*M_PI/3 || heading < -3*M_PI/4)
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading/4);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/4);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/4);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/4);
  }
  else if(heading > M_PI/2 || heading < -M_PI/2)
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading/3);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/3);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/3);
  }
  else if(heading > M_PI/4 || heading < -M_PI/4)
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading/2);
    step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/2);
  }
  else
  {
    step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading);
  }
  x_vector end_point_location;
  end_point_location.x = end_point[0];
  end_point_location.y = end_point[1];
  end_point_location.z = end_point[2];
  ROS_INFO("linking linear trajectory");
  step_list_msg = gait_generation.createLowerBodyLinearGaitQueue(step_list_msg, end_point_location, 0.0);
  step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
  if(end_point.size() == 4)
  {
	float final_heading = end_point[3]-total_heading;
    ROS_INFO("Pre Rotation: %f", final_heading);
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
    ROS_INFO("Successful Rotation: %f", final_heading);
    if(final_heading > 3*M_PI/4 || final_heading < -3*M_PI/4)
    {
      step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
      step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
      step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
      step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
    }
    else if(final_heading > M_PI/2 || final_heading < -M_PI/2)
    {
      step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/3);
      step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/3);
      step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/3);
    }
    else if(final_heading > M_PI/4 || final_heading < -M_PI/4)
    {
      step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/2);
      step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/2);
    }
    else
    {
      step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading);
    }
  }

  if(arms_disabled == false)
  {
    x_publisher.publish(gait_generation.createUpperBodyGait(step_list_msg));
  }
  ros::Duration(0.01).sleep(); // This sleep is necessary for Valkyrie to perform trajectories before leg movement
  x_publisher.publish(step_list_msg);
}

void
Path_Generation::traversePointList(std::vector<std::vector<float>> end_point, bool arms_disabled)
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
  ROS_INFO("Starting heading: %f", yaw);
  
  ihmc_msgs::FootstepDataListRosMessage step_list_msg;
  float heading = 0;
  float total_heading = yaw;
  for(int point_list_id = 0; point_list_id < end_point.size(); point_list_id++)
  {
    if(point_list_id == 0)
    {
      float x_distance_to_travel = end_point[point_list_id][0] - center_point.x;
      float y_distance_to_travel = end_point[point_list_id][1] - center_point.y;
      float z_distance_to_travel = end_point[point_list_id][2] - center_point.z;
      tf2::Matrix3x3 rotation;
      rotation.setRPY(0.0, 0.0, -total_heading);
      
      // Check if the points are behind valkyrie, thus prompting a rotation offset
      std::vector<float> point_comparison(3, 0);
      point_comparison[0] = x_distance_to_travel*rotation[0][0] + y_distance_to_travel*rotation[0][1] + z_distance_to_travel*rotation[0][2];
      point_comparison[1] = x_distance_to_travel*rotation[1][0] + y_distance_to_travel*rotation[1][1] + z_distance_to_travel*rotation[1][2];
      
      float distance = sqrt(pow(point_comparison[0], 2.0) + pow(point_comparison[1], 2.0));
      float new_heading = asin(point_comparison[1]/distance);
      ROS_INFO("XYOffset: %f\t%f", point_comparison[0], point_comparison[1]);
      if(point_comparison[0] <= 0)
      {
        if(point_comparison[1] <= 0)
        {
          total_heading += -M_PI-new_heading;
          heading = -M_PI-new_heading;
        }
        else
        {
          total_heading += M_PI-new_heading;
          heading = M_PI-new_heading;
        }
      }
      else
      {
        heading = new_heading;
        total_heading += new_heading;
      }
      ROS_INFO("Distance: %f\tHeading: %f\tRaw Heading: %f\tTotal Heading: %f", distance, heading, new_heading, total_heading);
      if(heading < -M_PI || heading > M_PI)
      {
        if(heading > 0)
        {
          heading -= 2*M_PI;
        }
        else
        {
          heading += 2*M_PI;
        }
      }
      if(total_heading < -M_PI || total_heading > M_PI)
      {
        if(total_heading > 0)
        {
          total_heading -= 2*M_PI;
        }
        else
        {
          total_heading += 2*M_PI;
        }
      }
      
      if(heading > 3*M_PI/4 || heading < -3*M_PI/4)
      {
        step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading/4);
        step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/4);
        step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/4);
        step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/4);
      }
      else if(heading > M_PI/2 || heading < -M_PI/2)
      {
        step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading/3);
        step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/3);
        step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/3);
      }
      else if(heading > M_PI/4 || heading < -M_PI/4)
      {
        step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading/2);
        step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, heading/2);
      }
      else
      {
        step_list_msg = gait_generation.createLowerBodyRotateGaitOverride(heading);
      }
      x_vector end_point_location;
      end_point_location.x = end_point[point_list_id][0];
      end_point_location.y = end_point[point_list_id][1];
      end_point_location.z = end_point[point_list_id][2];
      ROS_INFO("linking linear trajectory");
      step_list_msg = gait_generation.createLowerBodyLinearGaitQueue(step_list_msg, end_point_location, 0.0);
      step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
      if(end_point[point_list_id].size() == 4 && end_point.size() == point_list_id + 1)
      {
		float final_heading = end_point[point_list_id][3]-total_heading;
        ROS_INFO("Pre Rotation: %f", final_heading);
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
        ROS_INFO("Successful Rotation: %f", final_heading);
        if(final_heading > 3*M_PI/4 || final_heading < -3*M_PI/4)
        {
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
        }
        else if(final_heading > M_PI/2 || final_heading < -M_PI/2)
        {
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/3);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/3);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/3);
        }
        else if(final_heading > M_PI/4 || final_heading < -M_PI/4)
        {
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/2);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/2);
        }
        else
        {
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading);
        }
      }
    }
    else
    {
      // Sets heading to an inverse version of itself since
      // the new algorithm uses 0 degrees as its starting angle.
      heading = -heading;
      float x_distance_to_travel = end_point[point_list_id][0] - end_point[point_list_id - 1][0];
      float y_distance_to_travel = end_point[point_list_id][1] - end_point[point_list_id - 1][1];
      float z_distance_to_travel = end_point[point_list_id][2] - end_point[point_list_id - 1][2];
      tf2::Matrix3x3 rotation;
      rotation.setRPY(0.0, 0.0, -total_heading);
      
      // Check if the points are behind valkyrie, thus prompting a rotation offset
      std::vector<float> point_comparison(3, 0);
      point_comparison[0] = x_distance_to_travel*rotation[0][0] + y_distance_to_travel*rotation[0][1] + z_distance_to_travel*rotation[0][2];
      point_comparison[1] = x_distance_to_travel*rotation[1][0] + y_distance_to_travel*rotation[1][1] + z_distance_to_travel*rotation[1][2];
      
      float distance = sqrt(pow(point_comparison[0], 2.0) + pow(point_comparison[1], 2.0));
      float new_heading = asin(point_comparison[1]/distance);
      ROS_INFO("XYOffset: %f\t%f", point_comparison[0], point_comparison[1]);
      if(point_comparison[0] <= 0)
      {
        if(point_comparison[1] <= 0)
        {
          total_heading += -M_PI-new_heading;
          heading = -M_PI-new_heading;
        }
        else
        {
          total_heading += M_PI-new_heading;
          heading = M_PI-new_heading;
        }
      }
      else
      {
        heading = new_heading;
        total_heading += new_heading;
      }
      ROS_INFO("Distance: %f\tHeading: %f\tRaw Heading: %f\tTotal Heading: %f", distance, heading, new_heading, total_heading);
      if(heading < -M_PI || heading > M_PI)
      {
        if(heading > 0)
        {
          heading -= 2*M_PI;
        }
        else
        {
          heading += 2*M_PI;
        }
      }
      if(total_heading < -M_PI || total_heading > M_PI)
      {
        if(total_heading > 0)
        {
          total_heading -= 2*M_PI;
        }
        else
        {
          total_heading += 2*M_PI;
        }
      }
      
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
      x_vector end_point_location;
      end_point_location.x = end_point[point_list_id][0];
      end_point_location.y = end_point[point_list_id][1];
      end_point_location.z = end_point[point_list_id][2];
      ROS_INFO("linking linear trajectory");
      step_list_msg = gait_generation.createLowerBodyLinearGaitQueue(step_list_msg, end_point_location, 0.0);
      step_list_msg = gait_generation.createLowerBodyEndStepQueue(step_list_msg);
      if(end_point[point_list_id].size() == 4 && end_point.size() == point_list_id + 1)
      {
		float final_heading = end_point[point_list_id][3]-total_heading;
        ROS_INFO("Pre Rotation: %f", final_heading);
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
        ROS_INFO("Successful Rotation: %f", final_heading);
        if(final_heading > 3*M_PI/4 || final_heading < -3*M_PI/4)
        {
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/4);
        }
        else if(final_heading > M_PI/2 || final_heading < -M_PI/2)
        {
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/3);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/3);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/3);
        }
        else if(final_heading > M_PI/4 || final_heading < -M_PI/4)
        {
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/2);
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading/2);
        }
        else
        {
          step_list_msg = gait_generation.createLowerBodyRotateGaitQueue(step_list_msg, final_heading);
        }
      }
    }
  }

  if(arms_disabled == false)
  {
    x_publisher.publish(gait_generation.createUpperBodyGait(step_list_msg));
  }
  ros::Duration(0.01).sleep(); // This sleep is necessary for Valkyrie to perform trajectories before leg movement
  x_publisher.publish(step_list_msg);
}
