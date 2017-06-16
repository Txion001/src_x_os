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
#ifndef _PATH_GENERATION_HPP
#define _PATH_GENERATION_HPP

#include "ros/ros.h"
#include "map_scan/map_scan.hpp"
#include "gait_generation/gait_generation.hpp"
class Path_Generation
{
  private :
    ros::NodeHandle trajectory_handler;
    ros::Publisher scan_output;
    ros::Publisher stepListPublisher;
    ros::Publisher trajectoryListPublisher;
    
    Trajectory_Generation trajectory_generation;
    Gait_Generation gait_generation;
    
    const bool LEFT = 0;
    const bool RIGHT = 1;
    const std::string WORLD = "world";
    const std::string RIGHT_FOOT_ANKLE = "rightFoot";
    const std::string LEFT_FOOT_ANKLE = "leftFoot";
    const std::string RIGHT_FOOT_SOLE = "rightCOP_Frame";
    const std::string LEFT_FOOT_SOLE = "leftCOP_Frame";
    const std::string PELVIS = "pelvis";
    
    
  public :
    Path_Generation();
    
    float
    findMapEndDistance(pcl::PointCloud<pcl::PointXYZRGB>* map);
    std::vector<float>
    findMapEndPoint(pcl::PointCloud<pcl::PointXYZRGB>* map);
    // Finds a point that is a few distances forward and on the path.
    bool
    findMapNextPoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* next_point);
    std::vector<float>
    findMapNextPoint(pcl::PointCloud<pcl::PointXYZRGB>* map);
    std::vector<float>
    findMapNextPoint(pcl::PointCloud<pcl::PointXYZRGB>* map, float distance_to_travel);
    bool
    findTaskOnePoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output);
    bool
    findTaskTwoPanelPoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output);
    bool
    findTaskTwoArrayPoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output);
    bool
    findTaskTwoArrayPointClose(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output);
    bool
    findTaskThreeStairPoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output);
    std::vector<std::vector<float>>
    createPathPointList(tf2_ros::Buffer* tfBuffer, std::vector<float> end_point, pcl::PointCloud<pcl::PointXYZRGB>* map);
    std::vector<std::vector<float>>
    createPathPointHabList(tf2_ros::Buffer* tfBuffer, std::vector<float> end_point, pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    traverseToPoint(std::vector<float> end_point, bool arms_disabled);
    void
    traversePointList(std::vector<std::vector<float>> end_point, bool arms_disabled);
    bool
    findTaskThreeTablePoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output);
    bool
    findTaskThreeDetectorPoint(pcl::PointCloud<pcl::PointXYZRGB>* map, std::vector<float>* output);
};
 
#endif
