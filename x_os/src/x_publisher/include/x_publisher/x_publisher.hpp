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
#ifndef _X_PUBLISHER_HPP
#define _X_PUBLISHER_HPP

#include "ros/ros.h"
#include "ihmc_msgs/FootstepDataListRosMessage.h"
#include "ihmc_msgs/FootstepDataRosMessage.h"
#include "ihmc_msgs/HeadTrajectoryRosMessage.h"
#include "ihmc_msgs/NeckTrajectoryRosMessage.h"
#include "ihmc_msgs/WholeBodyTrajectoryRosMessage.h"
#include "ihmc_msgs/ArmTrajectoryRosMessage.h"
#include "ihmc_msgs/HandTrajectoryRosMessage.h"
#include "ihmc_msgs/ChestTrajectoryRosMessage.h"
#include "ihmc_msgs/PelvisTrajectoryRosMessage.h"
#include "ihmc_msgs/FootTrajectoryRosMessage.h"
#include "ihmc_msgs/OneDoFJointTrajectoryRosMessage.h"
#include "ihmc_msgs/TrajectoryPoint1DRosMessage.h"
#include "ihmc_msgs/Point2dRosMessage.h"
#include "ihmc_msgs/SO3TrajectoryPointRosMessage.h"
#include "ihmc_msgs/SE3TrajectoryPointRosMessage.h"
#include "ihmc_msgs/WalkingStatusRosMessage.h"
#include "std_msgs/Float64MultiArray.h"
#include "math.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2/LinearMath/Quaternion.h"

class X_Publisher
{
  private:
    ros::NodeHandle xPublisherNodeHandle;
    ros::CallbackQueue xPublisherCallback;
    ros::Publisher stepListPublisher;
    ros::Publisher trajectoryListPublisher;
    ros::Publisher trajectoryNeckPublisher;
    ros::Publisher trajectoryRightHandPublisher;
    ros::Publisher trajectoryLeftHandPublisher;
    
    ros::Subscriber footStepListener;
    
    bool steps_processed;
    
    void
    waitForFootSteps();
    void
    listenForFootSteps(const ihmc_msgs::WalkingStatusRosMessage& msg);
  
  public:
    X_Publisher();
    
    void
    publish(ihmc_msgs::FootstepDataListRosMessage step_list_msg);
    void
    publish(ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg);
    void
    publish(std_msgs::Float64MultiArray left_fingers, std_msgs::Float64MultiArray right_fingers);
    void
    publish(ihmc_msgs::NeckTrajectoryRosMessage neck_msg);
};

#endif
