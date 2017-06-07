/*
 * Designed by Tom Xiong
 * 
 * OBJECTIVE:
 * TO CONTROL THE X_OS
 * 
 * FORMAT:
 * MESSAGE & SERVICE
 * 
 * OWNERSHIP:
 * TEAM 1476-"XION SYSTEMS"
 * 
 * 
 */
#include "x_ocu/x_ocu_node.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "x_proto_node");
  ros::NodeHandle FCNodeHandle;
  
  commandClient = FCNodeHandle.serviceClient<x_msgs::X_Command>("x_os/x_command");
  robotStatusSubscriber = FCNodeHandle.subscribe("x_os/x_status", 1, statusCallback);
  
  ros::Rate rate(2);
  ROS_INFO("Connecting To Robot...");
  while(FCNodeHandle.ok() && !commandClient.exists())
  {
    if(!commandClient.exists())
    {
      rate.sleep();
      ROS_INFO("...");
    }
  }
  ROS_INFO("Connection Established");
  
  x_msgs::X_Command query_robot;
  query_robot.request.command = 0;
  
  std::string console_input = "Listen";
  while(FCNodeHandle.ok())
  {
    if(console_input == "Get Status")
    {
      query_robot.request.command = 0;
      if(commandClient.call(query_robot))
      {
        if(query_robot.response.status == 0)
        {
          std::cout << "SETUP MODE\n";
        }
        else if(query_robot.response.status == 1)
        {
          std::cout << "STANDBY MODE\n";
        }
        else if(query_robot.response.status == 2)
        {
          std::cout << "EXECUTE MODE\n";
        }
        else if(query_robot.response.status == 3)
        {
          std::cout << "SUCCESS MODE\n";
        }
        else if(query_robot.response.status == 4)
        {
          std::cout << "RESET MODE\n";
        }
      }
      else
      {
        std::cout << "ERROR COULD NOT QUERY ROBOT\n";
      }
    }
    else if(console_input == "Start Tasks")
    {
      query_robot.request.command = 1;
      if(commandClient.call(query_robot))
      {
        if(query_robot.response.status == 0)
        {
          std::cout << "SETUP MODE\n";
        }
        else if(query_robot.response.status == 1)
        {
          std::cout << "STANDBY MODE\n";
        }
        else if(query_robot.response.status == 2)
        {
          std::cout << "EXECUTE MODE\n";
        }
        else if(query_robot.response.status == 3)
        {
          std::cout << "SUCCESS MODE\n";
        }
        else if(query_robot.response.status == 4)
        {
          std::cout << "RESET MODE\n";
        }
      }
      else
      {
        std::cout << "ERROR COULD NOT QUERY ROBOT\n";
      }
    }
    else if(console_input == "Listen")
    {
      int robot_tmp_status = robot_status;
      while(FCNodeHandle.ok() == true && robot_tmp_status == robot_status)
      {
        ROS_INFO("Waiting for status update...");
        ros::spinOnce();
        rate.sleep();
      }
      if(robot_status == 0)
      {
        std::cout << "SETUP MODE\n";
      }
      else if(robot_status == 1)
      {
        std::cout << "STANDBY MODE\n";
      }
      else if(robot_status == 2)
      {
        std::cout << "EXECUTE MODE\n";
      }
      else if(robot_status == 3)
      {
        std::cout << "SUCCESS MODE\n";
      }
      else if(robot_status == 4)
      {
        std::cout << "RESET MODE\n";
      }
      
      else if(robot_status >= 30 && robot_status < 40)
      {
        std::cout << "EXECUTING TASK THREE: ";
        if(robot_status == 30)
        {
          std::cout << "WALKING TO STAIRS\n";
        }
      }
      else if(robot_status >= 20)
      {
        std::cout << "EXECUTING TASK TWO: ";
        if(robot_status == 20)
        {
          std::cout << "WALKING TO PANEL\n";
        }
        else if(robot_status == 21)
        {
          std::cout << "PICKED UP PANEL\n";
        }
        else if(robot_status == 22)
        {
          std::cout << "DROPPED PANEL\n";
        }
        else if(robot_status == 23)
        {
          std::cout << "OPENED PANEL\n";
        }
        else if(robot_status == 24)
        {
          std::cout << "GRABBED CABLE\n";
        }
        else if(robot_status == 25)
        {
          std::cout << "PLUGGED CABLE\n";
        }
      }
      else if(robot_status >= 10)
      {
        std::cout << "EXECUTING TASK ONE: ";
        if(robot_status == 10)
        {
          std::cout << "WALKING TO DISH CONSOLE\n";
        }
        else if(robot_status == 11)
        {
          std::cout << "ARRIVED AT DISH CONSOLE\n";
        }
        else if(robot_status == 13)
        {
          std::cout << "DISHED FIXED\n";
        }
      }
    }
    else if(console_input == "help")
    {
      std::cout << "HELP: Commands are:\n";
      std::cout << "Get Status\n";
      std::cout << "Start Tasks\n";
      std::cout << "Listen\n";
    }
    else
    {
      std::cout << "ERROR: COMMAND DOES NOT EXIST\n";
    }
    
    rate.sleep();
    
    std::cout << "\nPlease enter in a command to operate robot:\n";
    getline(std::cin, console_input);
  }
  return 0;
}

void
statusCallback(const x_msgs::X_Status& msg)
{
  robot_status = msg.status;
}
