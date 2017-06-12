#include "src_finals_controller/src_finals_controller.hpp"
#include "math.h"
#include "path_generation/path_generation.hpp"
#include "ihmc_msgs/ArmTrajectoryRosMessage.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "srcsim/Satellite.h"
#include "geometry_msgs/TransformStamped.h"
#include "satellite_dish_driver/satellite_dish_driver.hpp"
#include "solar_components_driver/solar_components_driver.hpp"
#include "habitat_driver/habitat_driver.hpp"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"
#include "x_msgs/X_Status.h"

Src_Finals_Controller::Src_Finals_Controller()
{
  ROS_INFO("Initalizing SRC Finals Controller...");
  scan_output = srcNodeHandle.advertise<sensor_msgs::PointCloud2>("/x_os", 1);
  ROS_INFO("Obtaining map...");
  this->map = map_scan.getMap();
  ROS_INFO("Storing joint positions in buffer...");
  ROS_INFO("Ros wait for 1.0 seconds to stablize");
  ros::Duration(1.0).sleep();
  ROS_INFO("Initialization Complete");
}

void
Src_Finals_Controller::exploreMap(tf2_ros::Buffer* tfBuffer)
{
  Path_Generation path_generation;
  bool map_explored = false;
  while(map_explored == false || path_generation.findMapEndDistance(&map) < 1.75)
  {
    map_scan.wideScan(&map);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
    
    std::vector<float> end_point;
    map_explored = !(path_generation.findMapNextPoint(&map, &end_point));
    path_generation.traversePointList(path_generation.createPathPointList(tfBuffer, end_point, &map), false);
  }
}

void
Src_Finals_Controller::exploreMap(tf2_ros::Buffer* tfBuffer, float distance_to_explore, bool arms_disabled)
{
  Path_Generation path_generation;
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  
  std::vector<float> end_point;
  end_point = path_generation.findMapNextPoint(&map, distance_to_explore);
  path_generation.traversePointList(path_generation.createPathPointList(tfBuffer, end_point, &map), arms_disabled);
}

void
Src_Finals_Controller::executeTaskOne(tf2_ros::Buffer* tfBuffer)
{
  Path_Generation path_generation;
  Satellite_Dish_Driver satellite_dish_driver;
  
  map_scan.wideScan(&map);
  map_scan.simpleTaskOneScan(&map);
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  
  std::vector<float> satellite_dish_point;
  while(path_generation.findTaskOnePoint(&map, &satellite_dish_point) == false)
  {
    exploreMap(tfBuffer, 3.0, false);
    
    map_scan.simpleTaskOneScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
  }
  std::vector<float> temp_map_end_point;
  temp_map_end_point = path_generation.findMapEndPoint(&map);
  path_generation.traversePointList(path_generation.createPathPointList(tfBuffer, satellite_dish_point, &map), false);
  satellite_dish_driver.grabTableBottom();
  ros::Duration(4.0).sleep();
  satellite_dish_driver.alignArms();
  satellite_dish_driver.checkTableSide();
  ros::Duration(2.0).sleep();
  satellite_dish_driver.centerBetweenArms();
  ros::Duration(2.0).sleep();
  satellite_dish_driver.grabTableSide();
  ros::Duration(2.0).sleep();
  satellite_dish_driver.alignDish();
  ros::Duration(2.0).sleep();
  satellite_dish_driver.letGoTable();
  ros::Duration(3.0).sleep();
  satellite_dish_driver.resetStance(temp_map_end_point);
  ros::Duration(4.0).sleep();
  ros::Duration(2.0);
  exploreMap(tfBuffer);
}

void
Src_Finals_Controller::executeTaskTwo(tf2_ros::Buffer* tfBuffer)
{
  Path_Generation path_generation;
  sensor_msgs::PointCloud2 output;
  Solar_Components_Driver solar_components_driver;
  ros::Duration(0.5).sleep();
  
  map_scan.wideScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  map_scan.wideTaskTwoHandleScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  
  std::vector<float> button_point;
  while(path_generation.findTaskTwoPanelPoint(&map, &button_point) == false)
  {
    ROS_INFO("Exploring map");
    exploreMap(tfBuffer, 2.0, false);
    
    map_scan.wideScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
    map_scan.wideTaskTwoHandleScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
  }
  
  std::vector<float> temp_map_end_point;
  temp_map_end_point = path_generation.findMapEndPoint(&map);
  path_generation.traversePointList(path_generation.createPathPointList(tfBuffer, button_point, &map), false);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  map_scan.lowTaskTwoHandleScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  solar_components_driver.alignToPanel(&map_scan, tfBuffer, &map);
  
  solar_components_driver.grabPanel(tfBuffer, &map);
  solar_components_driver.carryPanel();
  solar_components_driver.returnToPath(temp_map_end_point);
  
  map_scan.wideScan(&map);
  map_scan.wideTaskTwoCableScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  
  std::vector<float> cable_point;
  while(path_generation.findTaskTwoArrayPoint(&map, &cable_point) == false)
  {
    exploreMap(tfBuffer, 2.0, true);
    
    map_scan.wideScan(&map);
    map_scan.wideTaskTwoCableScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
  }
  temp_map_end_point = path_generation.findMapEndPoint(&map);
  path_generation.traversePointList(path_generation.createPathPointList(tfBuffer, cable_point, &map), true);
  
  map_scan.lowTaskTwoCableScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  path_generation.findTaskTwoArrayPointClose(&map, &cable_point);
  path_generation.traversePointList(path_generation.createPathPointList(tfBuffer, cable_point, &map), true);
  
  solar_components_driver.dropPanel();
  solar_components_driver.alignToArray(tfBuffer, &map);
  solar_components_driver.deployPanel(&map_scan, &map);
  solar_components_driver.grabCable();
  solar_components_driver.plugCable(&map_scan, tfBuffer, &map);
  solar_components_driver.resetStance(temp_map_end_point);
  exploreMap(tfBuffer);
}

void
Src_Finals_Controller::executeTaskThree(tf2_ros::Buffer* tfBuffer)
{
  sensor_msgs::PointCloud2 output;
  Path_Generation path_generation;
  Habitat_Driver habitat_driver;
  
  map_scan.wideScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  map_scan.wideTaskThreeStairScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  
  std::vector<float> stair_point;
  while(path_generation.findTaskThreeStairPoint(&map, &stair_point) == false)
  {
    exploreMap(tfBuffer, 2.0, true);
    
    map_scan.wideScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
    map_scan.wideTaskThreeStairScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
  }
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  path_generation.traversePointList(path_generation.createPathPointList(tfBuffer, stair_point, &map), false);
  
  habitat_driver.climbStairs(tfBuffer, &map);
  habitat_driver.walkToDoor(tfBuffer, &map);
  habitat_driver.openDoor();
  habitat_driver.enterHab();
}

void
Src_Finals_Controller::executeAllTasks(tf2_ros::Buffer* tfBuffer, ros::Publisher* statusPublisher)
{
  Path_Generation path_generation;
  sensor_msgs::PointCloud2 output;
  Satellite_Dish_Driver satellite_dish_driver;
  Solar_Components_Driver solar_components_driver;
  Habitat_Driver habitat_driver;
  x_msgs::X_Status current_status;
  ros::Duration(0.5).sleep();
  std::vector<float> temp_map_end_point;
  
  //Start Task One
  current_status.status = 10;
  statusPublisher->publish(current_status);
  map_scan.wideScan(&map);
  map_scan.simpleTaskOneScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  
  std::vector<float> satellite_dish_point;
  while(path_generation.findTaskOnePoint(&map, &satellite_dish_point) == false)
  {
    exploreMap(tfBuffer, 2.0, false);
    
    map_scan.wideScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
    map_scan.simpleTaskOneScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
  }
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  
  temp_map_end_point = path_generation.findMapEndPoint(&map);
  
  path_generation.traversePointList(path_generation.createPathPointList(tfBuffer, satellite_dish_point, &map), false);
  current_status.status = 11;
  statusPublisher->publish(current_status);
  satellite_dish_driver.grabTableBottom();
  ros::Duration(4.0).sleep();
  satellite_dish_driver.alignArms();
  satellite_dish_driver.checkTableSide();
  ros::Duration(2.0).sleep();
  satellite_dish_driver.centerBetweenArms();
  ros::Duration(2.0).sleep();
  satellite_dish_driver.grabTableSide();
  ros::Duration(2.0).sleep();
  satellite_dish_driver.alignDish();
  current_status.status = 13;
  statusPublisher->publish(current_status);
  ros::Duration(2.0).sleep();
  satellite_dish_driver.letGoTable();
  ros::Duration(3.0).sleep();
  satellite_dish_driver.resetStance(temp_map_end_point);
  ros::Duration(4.0).sleep();
  
  // Reach to next checkpoint
  map_scan.wideScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  exploreMap(tfBuffer, 3.5, false);
  
  //Start Task Two
  current_status.status = 20;
  statusPublisher->publish(current_status);
  
  map_scan.wideScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  map_scan.wideTaskTwoHandleScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  
  std::vector<float> button_point;
  while(path_generation.findTaskTwoPanelPoint(&map, &button_point) == false)
  {
    exploreMap(tfBuffer, 2.0, false);
    
    map_scan.wideScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
    map_scan.wideTaskTwoHandleScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
  }
  
  temp_map_end_point = path_generation.findMapEndPoint(&map);
  path_generation.traversePointList(path_generation.createPathPointList(tfBuffer, button_point, &map), false);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  map_scan.lowTaskTwoHandleScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  solar_components_driver.alignToPanel(&map_scan, tfBuffer, &map);
  
  solar_components_driver.grabPanel(tfBuffer, &map);
  solar_components_driver.carryPanel();
  current_status.status = 21;
  statusPublisher->publish(current_status);
  solar_components_driver.returnToPath(temp_map_end_point);
  
  map_scan.wideScan(&map);
  map_scan.wideTaskTwoCableScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  
  std::vector<float> cable_point;
  while(path_generation.findTaskTwoArrayPoint(&map, &cable_point) == false)
  {
    exploreMap(tfBuffer, 2.0, true);
    
    map_scan.wideScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
    map_scan.wideTaskTwoCableScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
  }
  temp_map_end_point = path_generation.findMapEndPoint(&map);
  path_generation.traversePointList(path_generation.createPathPointList(tfBuffer, cable_point, &map), true);
  
  map_scan.lowTaskTwoCableScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  path_generation.findTaskTwoArrayPointClose(&map, &cable_point);
  path_generation.traverseToPoint(cable_point, true);
  
  current_status.status = 22;
  statusPublisher->publish(current_status);
  solar_components_driver.alignToArray(tfBuffer, &map);
  solar_components_driver.dropPanel();
  solar_components_driver.deployPanel(&map_scan, &map);
  current_status.status = 23;
  statusPublisher->publish(current_status);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  solar_components_driver.grabCable();
  current_status.status = 24;
  statusPublisher->publish(current_status);
  solar_components_driver.plugCable(&map_scan, tfBuffer, &map);
  current_status.status = 25;
  statusPublisher->publish(current_status);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  solar_components_driver.resetStance(temp_map_end_point);
  
  current_status.status = 30;
  statusPublisher->publish(current_status);
  
  // Reach to next checkpoint
  map_scan.wideScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  exploreMap(tfBuffer, 3.5, false);
  
  // Start Task Three
  map_scan.wideScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  map_scan.wideTaskThreeStairScan(&map);
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  
  std::vector<float> stair_point;
  while(path_generation.findTaskThreeStairPoint(&map, &stair_point) == false)
  {
    exploreMap(tfBuffer, 2.0, true);
    
    map_scan.wideScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
    map_scan.wideTaskThreeStairScan(&map);
    pcl::toROSMsg(map, output);
    scan_output.publish(output);
  }
  pcl::toROSMsg(map, output);
  scan_output.publish(output);
  path_generation.traversePointList(path_generation.createPathPointList(tfBuffer, stair_point, &map), false);
  
  current_status.status = 31;
  statusPublisher->publish(current_status);
  
  habitat_driver.climbStairs(tfBuffer, &map);
  
  current_status.status = 32;
  statusPublisher->publish(current_status);
  
  habitat_driver.walkToDoor(tfBuffer, &map);
  habitat_driver.openDoor();
  habitat_driver.enterHab();
  
  current_status.status = 33;
  statusPublisher->publish(current_status);
}

void
Src_Finals_Controller::runController(tf2_ros::Buffer* tfBuffer, int task_id, ros::Publisher* statusPublisher)
{
  if(task_id == 0)
  {
    executeAllTasks(tfBuffer, statusPublisher);
  }
  else if(task_id == 1)
  {
    executeTaskOne(tfBuffer);
  }
  else if(task_id == 2)
  {
    executeTaskTwo(tfBuffer);
  }
  else if(task_id == 3)
  {
    executeTaskThree(tfBuffer);
  }
}
