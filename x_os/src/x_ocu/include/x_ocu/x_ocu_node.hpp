/*
 * Designed by Tom Xiong
 * 
 * OBJECTIVE:
 * TO ORGANIZE AND CONTROL OPERATION OF TASKS
 * 
 * OWNERSHIP:
 * TEAM 1476-"XION SYSTEMS"
 * 
 * 
 */
#ifndef _X_OCU_HPP
#define _X_OCU_HPP

#include "ros/ros.h"
#include "x_msgs/X_Status.h"
#include "x_msgs/X_Command.h"
#include <iostream>

ros::Subscriber robotStatusSubscriber;
ros::ServiceClient commandClient;

int robot_status;

void
statusCallback(const x_msgs::X_Status& msg);

#endif
