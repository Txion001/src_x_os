#include "trajectory_generation/trajectory_generation.hpp"
 
Trajectory_Generation::Trajectory_Generation()
{
}
 
ihmc_msgs::SO3TrajectoryPointRosMessage
Trajectory_Generation::createSO3TrajectoryOrient(float time, std::vector<float> orient, std::vector<float> ang_vel)
{
  ihmc_msgs::SO3TrajectoryPointRosMessage new_SO3_msg;
  
  new_SO3_msg.time = time;
  
  tf2Scalar roll = orient[0];
  tf2Scalar pitch = orient[1];
  tf2Scalar yaw = orient[2];
  tf2::Quaternion new_orientation;
  new_orientation.setRPY(roll, pitch, yaw);
  new_SO3_msg.orientation.x = new_orientation.x();
  new_SO3_msg.orientation.y = new_orientation.y();
  new_SO3_msg.orientation.z = new_orientation.z();
  new_SO3_msg.orientation.w = new_orientation.w();
  
  new_SO3_msg.angular_velocity.x = ang_vel[0];
  new_SO3_msg.angular_velocity.y = ang_vel[1];
  new_SO3_msg.angular_velocity.z = ang_vel[2];
  
  return new_SO3_msg;
}

ihmc_msgs::ArmTrajectoryRosMessage
Trajectory_Generation::appendTrajectoryPoint(float time, std::vector<float> trajectory, std::vector<float> ang_vel, ihmc_msgs::ArmTrajectoryRosMessage arm_msg)
{
  if(arm_msg.joint_trajectory_messages.size() == 0)
  {
    for(int joint_id = 0; joint_id < 7; joint_id++)
    {
      ihmc_msgs::OneDoFJointTrajectoryRosMessage new_joint;
      ihmc_msgs::TrajectoryPoint1DRosMessage new_trajectory_point;
      new_trajectory_point.time = time;
      new_trajectory_point.position = trajectory[joint_id];
      new_trajectory_point.velocity = ang_vel[joint_id];
      new_joint.trajectory_points.push_back(new_trajectory_point);
      arm_msg.joint_trajectory_messages.push_back(new_joint);
    }
  }
  else
  {
    for(int joint_id = 0; joint_id < 7; joint_id++)
    {
      ihmc_msgs::TrajectoryPoint1DRosMessage new_trajectory_point;
      new_trajectory_point.time = time;
      new_trajectory_point.position = trajectory[joint_id];
      new_trajectory_point.velocity = ang_vel[joint_id];
      arm_msg.joint_trajectory_messages[joint_id].trajectory_points.push_back(new_trajectory_point);
    }
  }
  return arm_msg;
}

std_msgs::Float64MultiArray
Trajectory_Generation::createFingerTrajectory(std::vector<float> joint_positions)
{
  std_msgs::Float64MultiArray joint_out;
  std_msgs::MultiArrayDimension temp_fingers;
  temp_fingers.label = "fingers";
  temp_fingers.size = 5;
  temp_fingers.stride = 5;
  joint_out.layout.dim.push_back(temp_fingers);
  joint_out.layout.data_offset = 0;
  for(int finger_motors_id = 0; finger_motors_id < joint_positions.size(); finger_motors_id++)
  {
    joint_out.data.push_back(joint_positions[finger_motors_id]);
  }
  return joint_out;
}

ihmc_msgs::NeckTrajectoryRosMessage
Trajectory_Generation::appendTrajectoryPoint(float time, std::vector<float> trajectory, std::vector<float> ang_vel, ihmc_msgs::NeckTrajectoryRosMessage neck_msg)
{
  if(neck_msg.joint_trajectory_messages.size() == 0)
  {
    for(int joint_id = 0; joint_id < 3; joint_id++)
    {
      ihmc_msgs::OneDoFJointTrajectoryRosMessage new_joint;
      ihmc_msgs::TrajectoryPoint1DRosMessage new_trajectory_point;
      new_trajectory_point.time = time;
      new_trajectory_point.position = trajectory[joint_id];
      new_trajectory_point.velocity = ang_vel[joint_id];
      new_joint.trajectory_points.push_back(new_trajectory_point);
      neck_msg.joint_trajectory_messages.push_back(new_joint);
    }
  }
  else
  {
    for(int joint_id = 0; joint_id < 3; joint_id++)
    {
      ihmc_msgs::TrajectoryPoint1DRosMessage new_trajectory_point;
      new_trajectory_point.time = time;
      new_trajectory_point.position = trajectory[joint_id];
      new_trajectory_point.velocity = ang_vel[joint_id];
      neck_msg.joint_trajectory_messages[joint_id].trajectory_points.push_back(new_trajectory_point);
    }
  }
  return neck_msg;
}

ihmc_msgs::SE3TrajectoryPointRosMessage
Trajectory_Generation::createPelvisHeightTrajectoryPoint(float time, float height)
{
  ihmc_msgs::SE3TrajectoryPointRosMessage new_SE3_msg;
  
  // Sets up buffer to recieve relative frame positions
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  geometry_msgs::TransformStamped pelvis = tfBuffer.lookupTransform(WORLD, PELVIS, ros::Time(0), ros::Duration(0.4));
  geometry_msgs::TransformStamped right_foot_sole = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_SOLE, ros::Time(0), ros::Duration(0.8));
  
  new_SE3_msg.time = time;
  
  new_SE3_msg.position.x = pelvis.transform.translation.x;
  new_SE3_msg.position.y = pelvis.transform.translation.y;
  new_SE3_msg.position.z = right_foot_sole.transform.translation.z + height;
  
  new_SE3_msg.orientation.x = pelvis.transform.rotation.x;
  new_SE3_msg.orientation.y = pelvis.transform.rotation.y;
  new_SE3_msg.orientation.z = pelvis.transform.rotation.z;
  new_SE3_msg.orientation.w = pelvis.transform.rotation.w;
  
  return new_SE3_msg;
}

ihmc_msgs::SE3TrajectoryPointRosMessage
Trajectory_Generation::createPelvisTrajectoryPointOffset(float time, std::vector<float> pos, std::vector<float> orient, std::vector<float> line_vel, std::vector<float> ang_vel)
{
  ihmc_msgs::SE3TrajectoryPointRosMessage new_SE3_msg;
  
  // Sets up buffer to recieve relative frame positions
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  geometry_msgs::TransformStamped pelvis = tfBuffer.lookupTransform(WORLD, PELVIS, ros::Time(0), ros::Duration(0.4));
  
  new_SE3_msg.time = time;
  
  tf2Scalar roll;
  tf2Scalar pitch;
  tf2Scalar yaw;
  tf2::Quaternion last_known_heading(pelvis.transform.rotation.x, pelvis.transform.rotation.y, pelvis.transform.rotation.z, pelvis.transform.rotation.w);
  tf2::Matrix3x3 last_heading(last_known_heading);
  last_heading.getRPY(roll, pitch, yaw);
  roll += orient[0];
  pitch += orient[1];
  yaw += orient[2];
  last_heading.setRPY(roll, pitch, yaw);
  new_SE3_msg.position.x = pelvis.transform.translation.x + pos[0]*last_heading[0][0] + pos[1]*last_heading[0][1] + pos[2]*last_heading[0][2];
  new_SE3_msg.position.y = pelvis.transform.translation.y + pos[0]*last_heading[1][0] + pos[1]*last_heading[1][1] + pos[2]*last_heading[1][2];
  new_SE3_msg.position.z = pelvis.transform.translation.z + pos[0]*last_heading[2][0] + pos[1]*last_heading[2][1] + pos[2]*last_heading[2][2];
  
  last_known_heading.setRPY(roll, pitch, yaw);
  
  new_SE3_msg.orientation.x = last_known_heading.x();
  new_SE3_msg.orientation.y = last_known_heading.y();
  new_SE3_msg.orientation.z = last_known_heading.z();
  new_SE3_msg.orientation.w = last_known_heading.w();
  
  new_SE3_msg.linear_velocity.x = line_vel[0];
  new_SE3_msg.linear_velocity.y = line_vel[0];
  new_SE3_msg.linear_velocity.z = line_vel[0];
  new_SE3_msg.angular_velocity.x = ang_vel[0];
  new_SE3_msg.angular_velocity.y = ang_vel[0];
  new_SE3_msg.angular_velocity.z = ang_vel[0];
  
  return new_SE3_msg;
}

ihmc_msgs::SE3TrajectoryPointRosMessage
Trajectory_Generation::createSE3(float time, std::vector<float> pos, std::vector<float> orient, std::vector<float> line_vel, std::vector<float> ang_vel)
{
  ihmc_msgs::SE3TrajectoryPointRosMessage new_SE3_msg;
  new_SE3_msg.time = time;
  new_SE3_msg.position.x = pos[0];
  new_SE3_msg.position.y = pos[1];
  new_SE3_msg.position.z = pos[2];
  tf2Scalar roll = orient[0];
  tf2Scalar pitch = orient[1];
  tf2Scalar yaw = orient[2];
  tf2::Quaternion new_orientation;
  new_orientation.setRPY(roll, pitch, yaw);
  new_SE3_msg.orientation.x = new_orientation.x();
  new_SE3_msg.orientation.y = new_orientation.y();
  new_SE3_msg.orientation.z = new_orientation.z();
  new_SE3_msg.orientation.w = new_orientation.w();
  new_SE3_msg.linear_velocity.x = line_vel[0];
  new_SE3_msg.linear_velocity.y = line_vel[1];
  new_SE3_msg.linear_velocity.z = line_vel[2];
  new_SE3_msg.angular_velocity.x = ang_vel[0];
  new_SE3_msg.angular_velocity.y = ang_vel[1];
  new_SE3_msg.angular_velocity.z = ang_vel[2];
  
  return new_SE3_msg;
}

ihmc_msgs::FootstepDataRosMessage
Trajectory_Generation::createStep(bool step_side, float step_distance, std::vector<float> pos_next_step)
{
  // Creates datastep msg
  ihmc_msgs::FootstepDataRosMessage new_step_msg;
  new_step_msg.origin = ihmc_msgs::FootstepDataRosMessage::AT_ANKLE_FRAME;
  //new_step_msg.trajectory_type = ihmc_msgs::FootstepDataRosMessage::OBSTACLE_CLEARANCE;
  //new_step_msg.swing_height = 0.1;
  //new_step_msg.has_timings = ihmc_msgs::FootstepDataRosMessage::DEFAULT;
  //new_step_msg.has_absolute_time = ihmc_msgs::FootstepDataRosMessage::DEFAULT;
  new_step_msg.unique_id = 1;
  
  // Sets up buffer to recieve relative frame positions
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  if(step_side == RIGHT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::RIGHT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.5 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.5 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.25 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.25 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
  }
  else if(step_side == LEFT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::LEFT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.25 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.25 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.5 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.5 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
  }
  new_step_msg.location.x = pos_next_step[0];
  new_step_msg.location.y = pos_next_step[1];
  new_step_msg.location.z = pos_next_step[2];
  
  tf2Scalar roll = 0;
  tf2Scalar pitch = 0;
  tf2Scalar yaw = 0;
  tf2::Quaternion new_orientation;
  new_orientation.setRPY(roll, pitch, yaw);
  new_step_msg.orientation.x = new_orientation.x();
  new_step_msg.orientation.y = new_orientation.y();
  new_step_msg.orientation.z = new_orientation.z();
  new_step_msg.orientation.w = new_orientation.w();
  
  return new_step_msg;
}

ihmc_msgs::FootstepDataRosMessage
Trajectory_Generation::createStepOffset(bool step_side, float step_distance, std::vector<float> pos_next_step)
{
  // Creates datastep msg
  ihmc_msgs::FootstepDataRosMessage new_step_msg;
  new_step_msg.origin = ihmc_msgs::FootstepDataRosMessage::AT_ANKLE_FRAME;
  new_step_msg.trajectory_type = ihmc_msgs::FootstepDataRosMessage::DEFAULT;
  new_step_msg.swing_height = 0.0;
  new_step_msg.has_timings = false;
  new_step_msg.has_absolute_time = true;
  new_step_msg.unique_id = 1;
  
  // Sets up buffer to recieve relative frame positions
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  if(step_side == RIGHT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::RIGHT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.5 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.5 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.25 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.25 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    
    geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
    geometry_msgs::TransformStamped pelvis = tfBuffer.lookupTransform(WORLD, PELVIS, ros::Time(0), ros::Duration(0.2));
    new_step_msg.location = right_foot_ankle.transform.translation;
    new_step_msg.orientation = pelvis.transform.rotation;
    
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = -0.06;
    if(yaw < -M_PI)
    {
      yaw += 2*M_PI;
    }
    else if(yaw > M_PI)
    {
      yaw -= 2*M_PI;
    }
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x = new_orientation.x();
    new_step_msg.orientation.y = new_orientation.y();
    new_step_msg.orientation.z = new_orientation.z();
    new_step_msg.orientation.w = new_orientation.w();
  }
  else if(step_side == LEFT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::LEFT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.25 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.25 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.5 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.5 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    
    geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
    geometry_msgs::TransformStamped pelvis = tfBuffer.lookupTransform(WORLD, PELVIS, ros::Time(0), ros::Duration(0.2));
    new_step_msg.location = left_foot_ankle.transform.translation;
    new_step_msg.orientation = pelvis.transform.rotation;
    
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = 0.06;
    if(yaw < -M_PI)
    {
      yaw += 2*M_PI;
    }
    else if(yaw > M_PI)
    {
      yaw -= 2*M_PI;
    }
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x = new_orientation.x();
    new_step_msg.orientation.y = new_orientation.y();
    new_step_msg.orientation.z = new_orientation.z();
    new_step_msg.orientation.w = new_orientation.w();
  }
  
  tf2::Quaternion old_orientation(new_step_msg.orientation.x, new_step_msg.orientation.y, new_step_msg.orientation.z, new_step_msg.orientation.w);
  tf2::Matrix3x3 rotation(old_orientation);
  new_step_msg.location.x += pos_next_step[0];
  new_step_msg.location.y += pos_next_step[1];
  new_step_msg.location.z += pos_next_step[2];
  
  return new_step_msg;
}

ihmc_msgs::FootstepDataRosMessage
Trajectory_Generation::createStepArc(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading)
{
  // Creates datastep msg
  ihmc_msgs::FootstepDataRosMessage new_step_msg;
  new_step_msg.origin = ihmc_msgs::FootstepDataRosMessage::AT_ANKLE_FRAME;
  new_step_msg.trajectory_type = ihmc_msgs::FootstepDataRosMessage::DEFAULT;
  new_step_msg.swing_height = 0.0;
  new_step_msg.has_timings = false;
  new_step_msg.has_absolute_time = true;
  new_step_msg.unique_id = 1;
  
  if(step_side == RIGHT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::RIGHT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.5 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.5 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.25 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.25 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = -0.06 + heading;
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x = new_orientation.x();
    new_step_msg.orientation.y = new_orientation.y();
    new_step_msg.orientation.z = new_orientation.z();
    new_step_msg.orientation.w = new_orientation.w();

  }
  else if(step_side == LEFT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::LEFT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.25 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.25 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.5 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.5 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = 0.06 + heading;
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x = new_orientation.x();
    new_step_msg.orientation.y = new_orientation.y();
    new_step_msg.orientation.z = new_orientation.z();
    new_step_msg.orientation.w = new_orientation.w();
  }
  new_step_msg.location.x += pos_next_step[0];
  new_step_msg.location.y += pos_next_step[1];
  new_step_msg.location.z += pos_next_step[2];
  
  return new_step_msg;
}

ihmc_msgs::FootstepDataRosMessage
Trajectory_Generation::createStepOffsetArc(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading)
{
  // Creates datastep msg
  ihmc_msgs::FootstepDataRosMessage new_step_msg;
  new_step_msg.origin = ihmc_msgs::FootstepDataRosMessage::AT_ANKLE_FRAME;
  new_step_msg.trajectory_type = ihmc_msgs::FootstepDataRosMessage::DEFAULT;
  new_step_msg.swing_height = 0.0;
  new_step_msg.has_timings = false;
  new_step_msg.has_absolute_time = true;
  new_step_msg.unique_id = 1;
  
  // Sets up buffer to recieve relative frame positions
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  if(step_side == RIGHT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::RIGHT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.5 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.5 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.25 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.25 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
    new_step_msg.location = right_foot_ankle.transform.translation;
    
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = -0.06 + heading;
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x = new_orientation.x();
    new_step_msg.orientation.y = new_orientation.y();
    new_step_msg.orientation.z = new_orientation.z();
    new_step_msg.orientation.w = new_orientation.w();

  }
  else if(step_side == LEFT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::LEFT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.25 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.25 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.5 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.5 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
    new_step_msg.location = left_foot_ankle.transform.translation;
    
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = 0.06 + heading;
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x = new_orientation.x();
    new_step_msg.orientation.y = new_orientation.y();
    new_step_msg.orientation.z = new_orientation.z();
    new_step_msg.orientation.w = new_orientation.w();
  }
  new_step_msg.location.x += pos_next_step[0];
  new_step_msg.location.y += pos_next_step[1];
  new_step_msg.location.z += pos_next_step[2];
  
  return new_step_msg;
}

ihmc_msgs::FootstepDataRosMessage
Trajectory_Generation::createStepLinear(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading)
{
  // Creates datastep msg
  ihmc_msgs::FootstepDataRosMessage new_step_msg;
  new_step_msg.trajectory_type = ihmc_msgs::FootstepDataRosMessage::DEFAULT;
  new_step_msg.swing_height = 0.0;
  new_step_msg.has_timings = false;
  new_step_msg.has_absolute_time = true;
  new_step_msg.unique_id = 1;
  
  if(step_side == RIGHT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::RIGHT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.5 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.5 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.125 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.125 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = -0.06 + heading;
    if(yaw < -M_PI)
    {
      yaw += 2*M_PI;
    }
    else if(yaw > M_PI)
    {
      yaw -= 2*M_PI;
    }
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x += new_orientation.x();
    new_step_msg.orientation.y += new_orientation.y();
    new_step_msg.orientation.z += new_orientation.z();
    new_step_msg.orientation.w += new_orientation.w();

  }
  else if(step_side == LEFT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::LEFT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.125 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.125 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.5 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.5 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = 0.06 + heading;
    if(yaw < -M_PI)
    {
      yaw += 2*M_PI;
    }
    else if(yaw > M_PI)
    {
      yaw -= 2*M_PI;
    }
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x += new_orientation.x();
    new_step_msg.orientation.y += new_orientation.y();
    new_step_msg.orientation.z += new_orientation.z();
    new_step_msg.orientation.w += new_orientation.w();
  }
  new_step_msg.location.x += pos_next_step[0];
  new_step_msg.location.y += pos_next_step[1];
  new_step_msg.location.z += pos_next_step[2];
  //ROS_INFO("RAWXYZ:\t%f\t%f\t%f", new_step_msg.location.x, new_step_msg.location.y, new_step_msg.location.z);
  
  return new_step_msg;
}

ihmc_msgs::FootstepDataRosMessage
Trajectory_Generation::createStepLinearStairs(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading)
{
  // Creates datastep msg
  ihmc_msgs::FootstepDataRosMessage new_step_msg;
  new_step_msg.trajectory_type = ihmc_msgs::FootstepDataRosMessage::OBSTACLE_CLEARANCE;
  new_step_msg.swing_height = 0.25;
  new_step_msg.has_timings = false;
  new_step_msg.has_absolute_time = true;
  new_step_msg.unique_id = 1;
  
  if(step_side == RIGHT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::RIGHT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.5 * FOOT_WIDTH;
    contact_points[1].x = -0.125 * FOOT_LENGTH;
    contact_points[1].y = 0.5 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.0 * FOOT_WIDTH;
    contact_points[3].x = -0.125 * FOOT_LENGTH;
    contact_points[3].y = -0.0 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = -0.06 + heading;
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x += new_orientation.x();
    new_step_msg.orientation.y += new_orientation.y();
    new_step_msg.orientation.z += new_orientation.z();
    new_step_msg.orientation.w += new_orientation.w();

  }
  else if(step_side == LEFT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::LEFT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.0 * FOOT_WIDTH;
    contact_points[1].x = -0.125 * FOOT_LENGTH;
    contact_points[1].y = 0.0 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.5 * FOOT_WIDTH;
    contact_points[3].x = -0.125 * FOOT_LENGTH;
    contact_points[3].y = -0.5 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = 0.06 + heading;
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x += new_orientation.x();
    new_step_msg.orientation.y += new_orientation.y();
    new_step_msg.orientation.z += new_orientation.z();
    new_step_msg.orientation.w += new_orientation.w();
  }
  new_step_msg.location.x += pos_next_step[0];
  new_step_msg.location.y += pos_next_step[1];
  new_step_msg.location.z += pos_next_step[2];
  //ROS_INFO("RAWXYZ:\t%f\t%f\t%f", new_step_msg.location.x, new_step_msg.location.y, new_step_msg.location.z);
  
  return new_step_msg;
}

ihmc_msgs::FootstepDataRosMessage
Trajectory_Generation::createStepOffsetLinear(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading)
{
  // Creates datastep msg
  ihmc_msgs::FootstepDataRosMessage new_step_msg;
  new_step_msg.origin = ihmc_msgs::FootstepDataRosMessage::AT_ANKLE_FRAME;
  new_step_msg.trajectory_type = ihmc_msgs::FootstepDataRosMessage::DEFAULT;
  new_step_msg.swing_height = 0.0;
  new_step_msg.has_timings = false;
  new_step_msg.has_absolute_time = true;
  new_step_msg.unique_id = 1;
  
  // Sets up buffer to recieve relative frame positions
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  if(step_side == RIGHT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::RIGHT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.5 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.5 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.25 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.25 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
    new_step_msg.location = right_foot_ankle.transform.translation;
    
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = -0.06 + heading;
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x += new_orientation.x();
    new_step_msg.orientation.y += new_orientation.y();
    new_step_msg.orientation.z += new_orientation.z();
    new_step_msg.orientation.w += new_orientation.w();

  }
  else if(step_side == LEFT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::LEFT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.25 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.25 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.5 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.5 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
    new_step_msg.location = left_foot_ankle.transform.translation;
    
    tf2Scalar roll = 0;
    tf2Scalar pitch = -step_distance/3;
    tf2Scalar yaw = 0.06 + heading;
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch, yaw);
    new_step_msg.orientation.x += new_orientation.x();
    new_step_msg.orientation.y += new_orientation.y();
    new_step_msg.orientation.z += new_orientation.z();
    new_step_msg.orientation.w += new_orientation.w();
  }
  new_step_msg.location.x += pos_next_step[0];
  new_step_msg.location.y += pos_next_step[1];
  new_step_msg.location.z += pos_next_step[2];
  
  return new_step_msg;
}

ihmc_msgs::FootstepDataRosMessage
Trajectory_Generation::createStepOffsetReset(bool step_side, float step_distance, std::vector<float> pos_next_step, float heading)
{
  // Creates datastep msg
  ihmc_msgs::FootstepDataRosMessage new_step_msg;
  new_step_msg.origin = ihmc_msgs::FootstepDataRosMessage::AT_ANKLE_FRAME;
  new_step_msg.trajectory_type = ihmc_msgs::FootstepDataRosMessage::DEFAULT;
  new_step_msg.swing_height = 0.0;
  new_step_msg.has_timings = false;
  new_step_msg.has_absolute_time = true;
  new_step_msg.unique_id = 1;
  
  // Sets up buffer to recieve relative frame positions
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
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
  
  
  if(step_side == RIGHT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::RIGHT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.5 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.5 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.25 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.25 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    new_step_msg.location = center_point;
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch - step_distance/3, yaw + heading - 0.06);
    
    new_step_msg.orientation.x = new_orientation.x();
    new_step_msg.orientation.y = new_orientation.y();
    new_step_msg.orientation.z = new_orientation.z();
    new_step_msg.orientation.w = new_orientation.w();

  }
  else if(step_side == LEFT)
  {
    new_step_msg.robot_side = ihmc_msgs::FootstepDataRosMessage::LEFT;
    
    std::vector<ihmc_msgs::Point2dRosMessage> contact_points(4);
    contact_points[0].x = 0.5 * FOOT_LENGTH;
    contact_points[0].y = 0.25 * FOOT_WIDTH;
    contact_points[1].x = -0.5 * FOOT_LENGTH;
    contact_points[1].y = 0.25 * FOOT_WIDTH;
    contact_points[2].x = 0.5 * FOOT_LENGTH;
    contact_points[2].y = -0.5 * FOOT_WIDTH;
    contact_points[3].x = -0.5 * FOOT_LENGTH;
    contact_points[3].y = -0.5 * FOOT_WIDTH;
    
    new_step_msg.predicted_contact_points = contact_points;
    new_step_msg.location = center_point;
    tf2::Quaternion new_orientation;
    new_orientation.setRPY(roll, pitch - step_distance/3, yaw + heading + 0.06);
    
    new_step_msg.orientation.x = new_orientation.x();
    new_step_msg.orientation.y = new_orientation.y();
    new_step_msg.orientation.z = new_orientation.z();
    new_step_msg.orientation.w = new_orientation.w();
  }
  new_step_msg.location.x += pos_next_step[0];
  new_step_msg.location.y += pos_next_step[1];
  new_step_msg.location.z += pos_next_step[2];
  
  return new_step_msg;
}
