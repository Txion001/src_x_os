/*
 * Designed by Tom Xiong
 * 
 * OBJECTIVE:
 * TO PROVIDE DYNAMIC CONTROL OF VALKYRIE'S WALKING THROUGH
 * UTILIZATION OF THE WHOLE BODY/FOOTSTEP CONTROLLERS
 * 
 * FORMAT:
 * DUAL STATE MACHINE
 * 
 * THREADS:
 * PASSIVE MACHINE
 * -DESCRIBES AVAILABILITY OF MACHINE
 * ACTIVE MACHINE
 * -DESCRIBES CURRENT PROCESS OF MACHINE
 * 
 * STATES:
 * PASSIVE
 * -SETUP
 * -READY
 * -UNAVAILABLE
 * -FAILURE
 * ACTIVE
 * -SETUP
 * -STANDBY
 * -EXECUTE
 * -SUCCESS
 * -RESET
 * 
 * OWNERSHIP:
 * TEAM 1476-"XION SYSTEMS"
 * 
 * 
 */

#include "x_main/x_main_node.hpp"
#include "x_publisher/x_publisher.hpp"
#include "srcsim/StartTask.h"

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "x_main_node");
  ros::NodeHandle mainNodeHandle;
  
  // Setup node handles for controlling callback queueing
  ros::NodeHandle trajectory_handler;
  ros::NodeHandle subscription_handler;
  ros::NodeHandle communication_handler;
    
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Duration(1.0).sleep();
  Src_Finals_Controller src_finals_controller;
  x_msgs::X_Status current_status;
  
  // Initialize dual state machine
  passive_state = PASSIVE_STATE[0];
  active_state = ACTIVE_STATE[0];
  while(mainNodeHandle.ok() == true && passive_state != PASSIVE_STATE[3]) // Maintain that the node has not been shutdown
  {
    // State machine for passive states
    if(passive_state == PASSIVE_STATE[0]) // SETUP MODE: Sets up controllers
    {
      // Console feedback of state status
      ROS_INFO("PASSIVE STATE: SETUP MODE");
      // Setup publishers & subscribers for feedback and control
      stepListPublisher = trajectory_handler.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list", 1);
      trajectoryListPublisher = trajectory_handler.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/whole_body_trajectory", 1);
      trajectoryNeckPublisher = trajectory_handler.advertise<ihmc_msgs::NeckTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/neck_trajectory", 1);
      trajectoryRightHandPublisher = trajectory_handler.advertise<std_msgs::Float64MultiArray>("/right_hand_position_controller/command", 1);
      trajectoryLeftHandPublisher = trajectory_handler.advertise<std_msgs::Float64MultiArray>("/left_hand_position_controller/command", 1);
      
      commandService = communication_handler.advertiseService("x_os/x_command", recieveCommand);
      statusPublisher = communication_handler.advertise<x_msgs::X_Status>("x_os/x_status", 1);
      tasksClient = communication_handler.serviceClient<srcsim::StartTask>("/srcsim/finals/start_task");
      
      ros::Rate rate(2);
      ROS_INFO("Connecting Topics");
      while(mainNodeHandle.ok() && (stepListPublisher.getNumSubscribers() == 0 || trajectoryListPublisher.getNumSubscribers() == 0))
      {
        if(stepListPublisher.getNumSubscribers() == 0 || trajectoryListPublisher.getNumSubscribers() == 0)
        {
          rate.sleep();
          ROS_INFO("...");
        }
      }
      ROS_INFO("Connection Established");
      
      current_status.status = 0;
      statusPublisher.publish(current_status);
      //Activate active_setup mode: Valkyrie moves into its ready state.
      active_state = ACTIVE_STATE[0];
      passive_state = PASSIVE_STATE[1];
    }
    else if(passive_state == PASSIVE_STATE[1]) // READY MODE
    {
      srcsim::StartTask task_one_service_start;
      task_one_service_start.request.task_id = 1;
      task_one_service_start.request.checkpoint_id = 1;
      tasksClient.call(task_one_service_start);
      // Console feedback of state status
      ROS_INFO("PASSIVE STATE: READY MODE");
      active_state = ACTIVE_STATE[2];
    }
    else if(passive_state == PASSIVE_STATE[2]) // PROCESSING MODE
    {
      if(current_command == 1 && active_state == ACTIVE_STATE[1])
      {
        ROS_INFO("Command Granted: Start All Tasks");
        active_state = ACTIVE_STATE[2];
      }
    }
    else if(passive_state == PASSIVE_STATE[3]) // FAILURE MODE
    {
      passive_state = PASSIVE_STATE[2];
    }
    // End of passive machine
    /*
     * ---------------------------------------------------------------
     */
    // State machine for active states
	
    if(active_state == ACTIVE_STATE[0]) // SETUP MODE
    {
      // Console feedback of state status
      ROS_INFO("ACTIVE STATE: SETUP MODE");
      current_status.status = 0;
      statusPublisher.publish(current_status);
      activeReset();
    }
    else if(active_state == ACTIVE_STATE[1]) // STANDBY MODE
    {
      // Console feedback of state status
      ROS_INFO("ACTIVE STATE: STANDBY MODE");
      srcsim::StartTask task_one_service_start;
      task_one_service_start.request.task_id = 1;
      task_one_service_start.request.checkpoint_id = 1;
      tasksClient.call(task_one_service_start);
      ros::Rate rate(2);
      while(mainNodeHandle.ok() == true && current_command == 0)
      {
        current_status.status = 1;
        statusPublisher.publish(current_status);
        ROS_INFO("Waiting for command...");
        ros::spinOnce();
        rate.sleep();
      }
      passive_state = PASSIVE_STATE[2];
    }
    else if(active_state == ACTIVE_STATE[2]) // EXECUTE MODE
    {
      current_status.status = 2;
      statusPublisher.publish(current_status);
      src_finals_controller.runController(&tfBuffer, 0, &statusPublisher);
      passive_state = PASSIVE_STATE[3];
      active_state = ACTIVE_STATE[3];
    }
    else if(active_state == ACTIVE_STATE[3]) // SUCCESS MODE
    {
      current_status.status = 3;
      statusPublisher.publish(current_status);
      passive_state = PASSIVE_STATE[1];
      active_state = ACTIVE_STATE[1];
    }
    else if(active_state == ACTIVE_STATE[4]) // RESET MODE
    {
      current_status.status = 4;
      statusPublisher.publish(current_status);
      active_state = ACTIVE_STATE[0];
    }
    // End of active machine
  }
  return 0;
}

bool
activeReset()
{
  X_Publisher x_publisher;
  x_publisher.publish(gait_generation.createStanceReset().trajectory_list_msg);
  x_publisher.publish(gait_generation.createStanceReset().trajectory_neck_msg);
  trajectoryLeftHandPublisher.publish(gait_generation.createStanceReset().trajectory_finger_left_msg);
  trajectoryRightHandPublisher.publish(gait_generation.createStanceReset().trajectory_finger_right_msg);
  ros::Duration(0.05).sleep(); // This sleep is necessary for Valkyrie to perform trajectories before leg movement
  x_publisher.publish(gait_generation.createStanceReset().step_list_msg);
}

bool
recieveCommand(x_msgs::X_Command::Request  &req, x_msgs::X_Command::Response &res)
{
  if(active_state == ACTIVE_STATE[0]) // SETUP MODE
  {
    res.status = 0;
  }
  else if(active_state == ACTIVE_STATE[1]) // STANDBY MODE
  {
    current_command = req.command;
    if(current_command == 1)
    {
      res.status = 2;
    }
    else
    {
      res.status = 1;
    }
      
  }
  else if(active_state == ACTIVE_STATE[2]) // EXECUTE MODE
  {
    res.status = 2;
  }
  else if(active_state == ACTIVE_STATE[3]) // SUCCESS MODE
  {
    res.status = 3;
  }
  else if(active_state == ACTIVE_STATE[4]) // RESET MODE
  {
    res.status = 4;
  }
}
