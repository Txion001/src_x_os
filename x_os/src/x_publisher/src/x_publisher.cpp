#include "x_publisher/x_publisher.hpp"

X_Publisher::X_Publisher()
{
  stepListPublisher = xPublisherNodeHandle.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list", 1);
  trajectoryListPublisher = xPublisherNodeHandle.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/whole_body_trajectory", 1);
  trajectoryNeckPublisher = xPublisherNodeHandle.advertise<ihmc_msgs::NeckTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/neck_trajectory", 1);
  trajectoryRightHandPublisher = xPublisherNodeHandle.advertise<std_msgs::Float64MultiArray>("/right_hand_position_controller/command", 1);
  trajectoryLeftHandPublisher = xPublisherNodeHandle.advertise<std_msgs::Float64MultiArray>("/left_hand_position_controller/command", 1);
  
  xPublisherNodeHandle.setCallbackQueue(&xPublisherCallback);
  footStepListener = xPublisherNodeHandle.subscribe("/ihmc_ros/valkyrie/output/walking_status", 1, &X_Publisher::listenForFootSteps, this);
  
  ros::Rate rate(2);
  ROS_INFO("Connecting X Publisher");
  while(xPublisherNodeHandle.ok() && (stepListPublisher.getNumSubscribers() == 0 || trajectoryListPublisher.getNumSubscribers() == 0))
  {
    if(stepListPublisher.getNumSubscribers() == 0 || trajectoryListPublisher.getNumSubscribers() == 0)
    {
      ROS_INFO("...");
      rate.sleep();
    }
  }
  ROS_INFO("Connection Established To X Publisher");
}

void
X_Publisher::waitForFootSteps()
{
  ros::Duration(1.0).sleep();
  this->steps_processed = false;
  ros::Rate rate(10);
  while(xPublisherNodeHandle.ok() && this->steps_processed == false)
  {
    this->steps_processed = false;
    xPublisherCallback.callAvailable(ros::WallDuration());
    rate.sleep();
  }
  ros::Duration(0.5).sleep();
}

void
X_Publisher::listenForFootSteps(const ihmc_msgs::WalkingStatusRosMessage& msg)
{
  if(msg.status == ihmc_msgs::WalkingStatusRosMessage::COMPLETED)
  {
    this->steps_processed = true;
  }
}

void
X_Publisher::publish(ihmc_msgs::FootstepDataListRosMessage step_list_msg)
{
  stepListPublisher.publish(step_list_msg);
  waitForFootSteps();
}
void
X_Publisher::publish(ihmc_msgs::WholeBodyTrajectoryRosMessage trajectory_list_msg)
{
  trajectoryListPublisher.publish(trajectory_list_msg);
}
void
X_Publisher::publish(std_msgs::Float64MultiArray left_fingers, std_msgs::Float64MultiArray right_fingers)
{
  trajectoryLeftHandPublisher.publish(left_fingers);
  trajectoryRightHandPublisher.publish(right_fingers);
}
void
X_Publisher::publish(ihmc_msgs::NeckTrajectoryRosMessage neck_msg)
{
  trajectoryNeckPublisher.publish(neck_msg);
}
