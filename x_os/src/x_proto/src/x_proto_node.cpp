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
#include "satellite_dish_driver/satellite_dish_driver.hpp"
#include "solar_components_driver/solar_components_driver.hpp"
#include "habitat_driver/habitat_driver.hpp"

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
  
  map = map_scan.getMap();
  
  Path_Generation path_generation;
  Satellite_Dish_Driver satellite_dish_driver;
  Solar_Components_Driver solar_components_driver;
  Habitat_Driver habitat_driver;
  
  map_scan.generateHabitat(&map);
  
  map_scan.wideTaskThreeStairScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  
  
  return 0;
}
