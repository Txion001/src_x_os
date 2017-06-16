/*
 * Designed by Tom Xiong
 * 
 * OBJECTIVE:
 * TO PROVIDE DYNAMIC MAPPING OF VALKYRIE"S SURROUNDINGS THROUGH
 * UTILIZATION OF THE NECK TRAJECTORY AND STEREO PROCESSING
 * 
 * OWNERSHIP:
 * TEAM 1476-"XION SYSTEMS"
 * 
 * TODO: Create scanning functions for task 1, 2, 3
 * 
 */
#ifndef _MAP_SCAN_HPP
#define _MAP_SCAN_HPP

#include "ros/ros.h"
#include "math.h"
#include "trajectory_generation/trajectory_generation.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"

class Map_Scan
{
  private:
    ros::NodeHandle cameraScanNodeHandle;
    ros::CallbackQueue camera_scan_callback;
    ros::Publisher trajectoryNeckPublisher;
    ros::Subscriber camera_sub;
    
    Trajectory_Generation trajectory_generation;
    
    const std::string WORLD = "world";
    const std::string CAMERA_FRAME = "left_camera_optical_frame";
    const std::string RIGHT_FOOT_SOLE = "rightCOP_Frame";
    const std::string RIGHT_FOOT_ANKLE = "rightFoot";
    const std::string LEFT_FOOT_ANKLE = "leftFoot";
    
    // Point Clouds for map scanning.
    pcl::PointCloud<pcl::PointXYZRGB> raw_scan;
    pcl::PointCloud<pcl::PointXYZRGB> raw_copy_scan;
    pcl::PointCloud<pcl::PointXYZRGB> map;
    pcl::PointCloud<pcl::PointXYZRGB> habitat;
  
    // Obtains the pointcloud image from topic
    void 
    rawScan(const sensor_msgs::PointCloud2ConstPtr& input);
    // Downsamples cloud points into a grid
    pcl::PointCloud<pcl::PointXYZRGB>
    voxelGridFilter(std::vector<float> leaf_size, pcl::PointCloud<pcl::PointXYZRGB>* input);
    // Filters noise in cloud
    pcl::PointCloud<pcl::PointXYZRGB>
    noiseFilter(std::vector<float> leaf_size, pcl::PointCloud<pcl::PointXYZRGB>* input);
    // Filters obstacles that hang over floor
    void
    overHangFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    overHangBlueFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    // Filters raw point cloud to floor
    void
    floorFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output);
    void
    floorHabFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output);
    void
    taskOneFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output);
    void
    taskTwoFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output);
    void
    taskThreeFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output);
    void
    taskTwoHandleFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output, std::vector<float> button_point);
    void
    taskTwoArrayFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output, std::vector<float> cable_point);
    bool
    RedFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    RedRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    bool
    BlackFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    BlackRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    bool
    BlueFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    BlueRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    GoldRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    bool
    GoldFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    YellowRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    bool
    HandleCorrectFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    bool
    CableCorrectFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    bool
    LightBlueFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    LightBlueRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    bool
    TealFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    TealRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    overHangHabFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    overHangTaskThreeFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
  
  public:
    Map_Scan();
    /*
     * Performs scans of various types:
     * WideScan: Complete scan by turning head to left, right, down, then default for scans.
     * NarrowScan: Scans default and down head positions
     * SimpleScan: Scans default head position only
     */
    pcl::PointCloud<pcl::PointXYZRGB>
    getMap();
    pcl::PointCloud<pcl::PointXYZRGB>
    getOverHangMap();

    void
    wideScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    forwardScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    lowScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    leftScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    rightScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    wideHabScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    forwardHabScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    lowHabScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    leftHabScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    rightHabScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    narrowScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    simpleScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    simpleTaskOneScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    simpleTaskTwoScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    simpleTaskTwoHandleScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    wideTaskTwoHandleScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    simpleTaskThreeHandleScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    wideTaskThreeComponentsScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    simpleTaskTwoCableScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    wideTaskTwoCableScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    lowTaskTwoHandleScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    lowTaskTwoCableScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    lowTaskTwoPlugScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    wideTaskThreeStairScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    void
    wideLowTaskThreeStairScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
    bool
    YellowFilter(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    generateHabitat(pcl::PointCloud<pcl::PointXYZRGB>* input);
    void
    lowTaskThreeScan(pcl::PointCloud<pcl::PointXYZRGB>* map);
};

#endif
