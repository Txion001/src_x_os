#include "map_scan/map_scan.hpp"

Map_Scan::Map_Scan()
{
  cameraScanNodeHandle.setCallbackQueue(&camera_scan_callback);
  camera_sub = cameraScanNodeHandle.subscribe("/multisense/image_points2_color", 2, &Map_Scan::rawScan, this);
  trajectoryNeckPublisher = cameraScanNodeHandle.advertise<ihmc_msgs::NeckTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/neck_trajectory", 1);
  ros::Rate rate(2);
  ROS_INFO("Connecting map_scan...");
  while(cameraScanNodeHandle.ok() && trajectoryNeckPublisher.getNumSubscribers() == 0)
  {
    if(trajectoryNeckPublisher.getNumSubscribers() == 0)
    {
      rate.sleep();
      ROS_INFO("...");
    }
  }
  ROS_INFO("Connection Established");
  tf2_ros::Buffer tfBuffer(ros::Duration(1.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped right_foot_sole = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_SOLE, ros::Time(0), ros::Duration(0.8));
  
  this->map.header.frame_id = WORLD;
  
  // Generate starting point
  pcl::PointXYZRGB ground_space;
  for(float x = -1.5; x < 1.5; x += 0.15)
  {
    ground_space.x = x;
    for(float y = -1.5; y < 1.5; y += 0.15)
    {
      if(x > 1.4 || x < -1.4 || y > 1.4 || y < -1.4)
      {
        ground_space.r = 150;
        ground_space.g = 150;
        ground_space.b = 150;
      }
      else
      {
        ground_space.r = 30;
        ground_space.g = 30;
        ground_space.b = 30;
      }
      ground_space.y = y;
      ground_space.z = right_foot_sole.transform.translation.z;
      this->map.points.push_back(ground_space);
    }
  }
}

void
Map_Scan::generateHabitat(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(2.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped right_foot_sole = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_SOLE, ros::Time(0), ros::Duration(0.8));
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer.lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer.lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
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
  
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  
  ROS_INFO("heading: %f", yaw);
  
  int num_angle = round(yaw / M_PI_4);
  float heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", heading);
  
  std::vector<float> end_point(3, 0);
  end_point[0] = 5.0;
  end_point[1] = 0.0;
  end_point[2] = 0.0;
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, heading);
  
  std::vector<float> task_point(3, 0);
  task_point[0] = (end_point[0]*rotation[0][0] + end_point[1]*rotation[0][1] + end_point[2]*rotation[0][2]);
  task_point[1] = (end_point[0]*rotation[1][0] + end_point[1]*rotation[1][1] + end_point[2]*rotation[1][2]);
  task_point[2] = (end_point[0]*rotation[2][0] + end_point[1]*rotation[2][1] + end_point[2]*rotation[2][2]);
  
  ROS_INFO("XYZ location:\t%f\t%f\t%f\tHeading: %f", task_point[0], task_point[1], task_point[2], heading);
  task_point[0] = task_point[0] + center_point.x;
  task_point[1] = task_point[1] + center_point.y;
  task_point[2] = task_point[2] + center_point.z;
  ROS_INFO("XYZ location:\t%f\t%f\t%f\tHeading: %f", task_point[0], task_point[1], task_point[2], heading);
  // Generate habitat floor plan
  // Generate starting point
  pcl::PointXYZRGB ground_space;
  for(float x = -10; x < 10; x += 0.15)
  {
    ground_space.x = x;
    for(float y = -10; y < 10; y += 0.15)
    {
      ground_space.y = y;
      bool is_out_bounds = false;
      for(float angle = heading; angle < 2*M_PI+heading && is_out_bounds == false; angle += M_PI/3)
      {
        tf2::Matrix3x3 new_rotation;
        new_rotation.setRPY(0.0, 0.0, angle);
        float point_distance = ground_space.x*new_rotation[0][0] + ground_space.y*new_rotation[0][1];
        if(point_distance > 5)
        {
          is_out_bounds = true;
        }
      }
      if(is_out_bounds == false)
      {
        pcl::PointXYZRGB tmp_space;
        tmp_space.x = task_point[0] + ground_space.x;
        tmp_space.y = task_point[1] + ground_space.y;
        tmp_space.z = right_foot_sole.transform.translation.z;
        tmp_space.r = 30;
        tmp_space.g = 30;
        tmp_space.b = 30;
        input->points.push_back(tmp_space);
      }
    }
  }
}

pcl::PointCloud<pcl::PointXYZRGB>
Map_Scan::getMap()
{
  return this->map;
}
pcl::PointCloud<pcl::PointXYZRGB>
Map_Scan::getOverHangMap()
{
  return this->raw_copy_scan;
}

void 
Map_Scan::rawScan(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::fromROSMsg(*input, this->raw_scan);
}

pcl::PointCloud<pcl::PointXYZRGB>
Map_Scan::noiseFilter(std::vector<float> leaf_size, pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  
  cloud.header = input->header;
  
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    int cloud_new_input_point = 0;
    for(unsigned long cloud_point_id = 0; cloud_point_id < input->points.size() && cloud_new_input_point <= 10; cloud_point_id++)
    {
      bool x_comparison = (input->points[cloud_point_id].x > input->points[point_id].x - leaf_size[0]) && (input->points[cloud_point_id].x < input->points[point_id].x + leaf_size[0]);
      bool y_comparison = (input->points[cloud_point_id].y > input->points[point_id].y - leaf_size[1]) && (input->points[cloud_point_id].y < input->points[point_id].y + leaf_size[1]);
      bool z_comparison = (input->points[cloud_point_id].z > input->points[point_id].z - leaf_size[2]) && (input->points[cloud_point_id].z < input->points[point_id].z + leaf_size[2]);
      if(x_comparison && y_comparison && z_comparison)
      {
        cloud_new_input_point++;
      }
    }
    if(cloud_new_input_point > 10)
    {
      cloud.points.push_back(input->points[point_id]);
    }
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>
Map_Scan::voxelGridFilter(std::vector<float> leaf_size, pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  
  cloud.header = input->header;
  
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    input->points[point_id].x = round(input->points[point_id].x/leaf_size[0]) * leaf_size[0];
    input->points[point_id].y = round(input->points[point_id].y/leaf_size[1]) * leaf_size[1];
    if(point_id == 0)
    {
      cloud.points.push_back(input->points[point_id]);
    }
    else
    {
      unsigned long cloud_size = cloud.points.size();
      bool cloud_new_input_point = true;
      for(unsigned long cloud_point_id = 0; cloud_point_id < cloud_size && cloud_new_input_point == true; cloud_point_id++)
      {
        bool x_comparison = (cloud.points[cloud_point_id].x > input->points[point_id].x - leaf_size[0]/2) && (cloud.points[cloud_point_id].x < input->points[point_id].x + leaf_size[0]/2);
        bool y_comparison = (cloud.points[cloud_point_id].y > input->points[point_id].y - leaf_size[1]/2) && (cloud.points[cloud_point_id].y < input->points[point_id].y + leaf_size[1]/2);
        bool z_comparison = (cloud.points[cloud_point_id].z > input->points[point_id].z - leaf_size[2]/2) && (cloud.points[cloud_point_id].z < input->points[point_id].z + leaf_size[2]/2);
        if(x_comparison && y_comparison && z_comparison)
        {
          cloud_new_input_point = false;
        }
      }
      if(cloud_new_input_point == true)
      {
        cloud.points.push_back(input->points[point_id]);
      }
    }
  }
  return cloud;
}

void
Map_Scan::RedRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g < 50 && input->points[point_id].r > 100 && input->points[point_id].b < 50)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
}

bool
Map_Scan::RedFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointXYZRGB cloud_point;
  cloud_point.x = 0.0;
  cloud_point.y = 0.0;
  cloud_point.z = 0.0;
  
  int numRedPoints = 0;
  
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g < 50 && input->points[point_id].r > 100 && input->points[point_id].b < 50)
    {
      cloud_point.x += input->points[point_id].x;
      cloud_point.y += input->points[point_id].y;
      cloud_point.z += input->points[point_id].z;
      numRedPoints++;
    }
  }
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g < 50 && input->points[point_id].r > 100 && input->points[point_id].b < 50)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
  
  cloud_point.x = cloud_point.x / numRedPoints;
  cloud_point.y = cloud_point.y / numRedPoints;
  cloud_point.z = cloud_point.z / numRedPoints;
  cloud_point.r = 200;
  cloud_point.g = 0;
  cloud_point.b = 0;
  
  if(numRedPoints > 0)
  {
    input->points.push_back(cloud_point);
    return true;
  }
  else
  {
    return false;
  }
}

void
Map_Scan::BlackRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 10 && input->points[point_id].r == 10 && input->points[point_id].b == 10)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
}

bool
Map_Scan::BlackFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointXYZRGB cloud_point;
  cloud_point.x = 0.0;
  cloud_point.y = 0.0;
  cloud_point.z = 0.0;
  
  int numBlackPoints = 0;
  
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 10 && input->points[point_id].r == 10 && input->points[point_id].b == 10)
    {
      cloud_point.x += input->points[point_id].x;
      cloud_point.y += input->points[point_id].y;
      cloud_point.z += input->points[point_id].z;
      numBlackPoints++;
    }
  }
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 10 && input->points[point_id].r == 10 && input->points[point_id].b == 10)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
  
  cloud_point.x = cloud_point.x / numBlackPoints;
  cloud_point.y = cloud_point.y / numBlackPoints;
  cloud_point.z = cloud_point.z / numBlackPoints;
  cloud_point.r = 10;
  cloud_point.g = 10;
  cloud_point.b = 10;
  
  if(numBlackPoints > 0)
  {
    input->points.push_back(cloud_point);
    return true;
  }
  else
  {
    return false;
  }
}

void
Map_Scan::BlueRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g < 50 && input->points[point_id].b > 80 && input->points[point_id].r < 50)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
}

bool
Map_Scan::BlueFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointXYZRGB cloud_point;
  cloud_point.x = 0.0;
  cloud_point.y = 0.0;
  cloud_point.z = 0.0;
  
  int numBluePoints = 0;
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 0 && input->points[point_id].b == 200 && input->points[point_id].r == 0)
    {
      cloud_point.x += input->points[point_id].x;
      cloud_point.y += input->points[point_id].y;
      cloud_point.z += input->points[point_id].z;
      numBluePoints++;
    }
  }
  ROS_INFO("Num Points: %i", numBluePoints);
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 0 && input->points[point_id].b == 200 && input->points[point_id].r == 0)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
  cloud_point.x = cloud_point.x / numBluePoints;
  cloud_point.y = cloud_point.y / numBluePoints;
  cloud_point.z = cloud_point.z / numBluePoints;
  cloud_point.r = 0;
  cloud_point.g = 0;
  cloud_point.b = 200;
  
  if(numBluePoints > 0)
  {
    input->points.push_back(cloud_point);
    return true;
  }
  else
  {
    return false;
  }
}

void
Map_Scan::LightBlueRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 100 && input->points[point_id].b == 200 && input->points[point_id].r == 0)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
}

bool
Map_Scan::LightBlueFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointXYZRGB cloud_point;
  cloud_point.x = 0.0;
  cloud_point.y = 0.0;
  cloud_point.z = 0.0;
  
  int numBluePoints = 0;
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 100 && input->points[point_id].b == 200 && input->points[point_id].r == 0)
    {
      cloud_point.x += input->points[point_id].x;
      cloud_point.y += input->points[point_id].y;
      cloud_point.z += input->points[point_id].z;
      numBluePoints++;
    }
  }
  ROS_INFO("Num Points: %i", numBluePoints);
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 100 && input->points[point_id].b == 200 && input->points[point_id].r == 0)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
  cloud_point.x = cloud_point.x / numBluePoints;
  cloud_point.y = cloud_point.y / numBluePoints;
  cloud_point.z = cloud_point.z / numBluePoints;
  cloud_point.r = 0;
  cloud_point.g = 100;
  cloud_point.b = 200;
  
  if(numBluePoints > 0)
  {
    input->points.push_back(cloud_point);
    return true;
  }
  else
  {
    return false;
  }
}

void
Map_Scan::TealRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 200 && input->points[point_id].b == 200 && input->points[point_id].r == 50)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
}

bool
Map_Scan::TealFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointXYZRGB cloud_point;
  cloud_point.x = 0.0;
  cloud_point.y = 0.0;
  cloud_point.z = 0.0;
  
  int numBluePoints = 0;
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 200 && input->points[point_id].b == 200 && input->points[point_id].r == 50)
    {
      cloud_point.x += input->points[point_id].x;
      cloud_point.y += input->points[point_id].y;
      cloud_point.z += input->points[point_id].z;
      numBluePoints++;
    }
  }
  ROS_INFO("Num Points: %i", numBluePoints);
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 200 && input->points[point_id].b == 200 && input->points[point_id].r == 50)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
  cloud_point.x = cloud_point.x / numBluePoints;
  cloud_point.y = cloud_point.y / numBluePoints;
  cloud_point.z = cloud_point.z / numBluePoints;
  cloud_point.r = 50;
  cloud_point.g = 200;
  cloud_point.b = 200;
  
  if(numBluePoints > 0)
  {
    input->points.push_back(cloud_point);
    return true;
  }
  else
  {
    return false;
  }
}

bool
Map_Scan::HandleCorrectFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointXYZRGB handle_point;
  handle_point.x = 0.0;
  handle_point.y = 0.0;
  handle_point.z = 0.0;
  handle_point.r = 0;
  handle_point.g = 0;
  handle_point.b = 200;
  pcl::PointXYZRGB button_point;
  button_point.x = 0.0;
  button_point.y = 0.0;
  button_point.z = 0.0;
  button_point.r = 200;
  button_point.g = 0;
  button_point.b = 0;
  
  int numBluePoints = 0;
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 0 && input->points[point_id].b == 200 && input->points[point_id].r == 0)
    {
      handle_point.x += input->points[point_id].x;
      handle_point.y += input->points[point_id].y;
      handle_point.z += input->points[point_id].z;
      numBluePoints++;
    }
    else if(input->points[point_id].g < 50 && input->points[point_id].r > 100 && input->points[point_id].b < 50)
    {
      button_point.x += input->points[point_id].x;
      button_point.y += input->points[point_id].y;
      button_point.z += input->points[point_id].z;
    }
  }
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 0 && input->points[point_id].b == 200 && input->points[point_id].r == 0)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
  
  handle_point.x = handle_point.x / numBluePoints;
  handle_point.y = handle_point.y / numBluePoints;
  handle_point.z = handle_point.z / numBluePoints;
  
  float button_handle_seperation = sqrt(pow(fabs(handle_point.x - button_point.x), 2.0) + pow(fabs(handle_point.y - button_point.y), 2.0));
  float heading = asin((button_point.y - handle_point.y)/button_handle_seperation);
  if(button_point.x - handle_point.x <= 0)
  {
    if(button_point.y - handle_point.y <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  
  std::vector<float> handle_offset_point(3, 0);
  handle_offset_point[0] = -0.16;
  handle_offset_point[1] = 0.0;
  handle_offset_point[2] = 0.0;
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, heading);
  
  handle_point.x = handle_offset_point[0]*rotation[0][0] + handle_offset_point[1]*rotation[0][1] + handle_offset_point[2]*rotation[0][2] + button_point.x;
  handle_point.y = handle_offset_point[0]*rotation[1][0] + handle_offset_point[1]*rotation[1][1] + handle_offset_point[2]*rotation[1][2] + button_point.y;
  handle_point.z = handle_offset_point[0]*rotation[2][0] + handle_offset_point[1]*rotation[2][1] + handle_offset_point[2]*rotation[2][2] + handle_point.z;
  
  if(numBluePoints > 0)
  {
    input->points.push_back(handle_point);
    return true;
  }
  else
  {
    return false;
  }
}

bool
Map_Scan::CableCorrectFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointXYZRGB plug_point;
  plug_point.x = 0.0;
  plug_point.y = 0.0;
  plug_point.z = 0.0;
  plug_point.r = 0;
  plug_point.g = 0;
  plug_point.b = 200;
  pcl::PointXYZRGB cable_point;
  cable_point.x = 0.0;
  cable_point.y = 0.0;
  cable_point.z = 0.0;
  cable_point.r = 200;
  cable_point.g = 0;
  cable_point.b = 0;
  
  int numRedPoints = 0;
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 0 && input->points[point_id].b == 0 && input->points[point_id].r == 200)
    {
      cable_point.x += input->points[point_id].x;
      cable_point.y += input->points[point_id].y;
      cable_point.z += input->points[point_id].z;
      numRedPoints++;
    }
    else if(input->points[point_id].g == 0 && input->points[point_id].b == 200 && input->points[point_id].r == 0)
    {
      plug_point.x += input->points[point_id].x;
      plug_point.y += input->points[point_id].y;
      plug_point.z += input->points[point_id].z;
    }
  }
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 0 && input->points[point_id].b == 0 && input->points[point_id].r == 200)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
  
  cable_point.x = cable_point.x / numRedPoints;
  cable_point.y = cable_point.y / numRedPoints;
  cable_point.z = cable_point.z / numRedPoints;
  
  float plug_cable_seperation = sqrt(pow(fabs(plug_point.x - cable_point.x), 2.0) + pow(fabs(plug_point.y - cable_point.y), 2.0));
  float heading = asin((plug_point.y - cable_point.y)/plug_cable_seperation);
  if(plug_point.x - cable_point.x <= 0)
  {
    if(plug_point.y - cable_point.y <= 0)
    {
      heading = -M_PI-heading;
    }
    else
    {
      heading = M_PI-heading;
    }
  }
  
  int num_angle = round(heading / M_PI_4);
  heading = num_angle * M_PI_4;
  ROS_INFO("Corrected Heading: %f", heading);
  
  std::vector<float> cable_offset_point(3, 0);
  cable_offset_point[0] = -0.25;
  cable_offset_point[1] = 0.0;
  cable_offset_point[2] = 0.0;
  tf2::Matrix3x3 rotation;
  rotation.setRPY(0.0, 0.0, heading);
  
  cable_point.x = cable_offset_point[0]*rotation[0][0] + cable_offset_point[1]*rotation[0][1] + cable_offset_point[2]*rotation[0][2] + plug_point.x;
  cable_point.y = cable_offset_point[0]*rotation[1][0] + cable_offset_point[1]*rotation[1][1] + cable_offset_point[2]*rotation[1][2] + plug_point.y;
  cable_point.z = cable_offset_point[0]*rotation[2][0] + cable_offset_point[1]*rotation[2][1] + cable_offset_point[2]*rotation[2][2] + plug_point.z;
  
  if(numRedPoints > 0)
  {
    input->points.push_back(cable_point);
    return true;
  }
  else
  {
    return false;
  }
}

void
Map_Scan::GoldRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g > 100 && input->points[point_id].r > 100 && input->points[point_id].b < 50)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
}

bool
Map_Scan::GoldFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointXYZRGB cloud_point;
  cloud_point.x = 0.0;
  cloud_point.y = 0.0;
  cloud_point.z = 0.0;
  
  int numBluePoints = 0;
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g > 100 && input->points[point_id].r > 100 && input->points[point_id].b < 50)
    {
      cloud_point.x += input->points[point_id].x;
      cloud_point.y += input->points[point_id].y;
      cloud_point.z += input->points[point_id].z;
      numBluePoints++;
    }
  }
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g > 100 && input->points[point_id].r > 100 && input->points[point_id].b < 50)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
  
  cloud_point.x = cloud_point.x / numBluePoints;
  cloud_point.y = cloud_point.y / numBluePoints;
  cloud_point.z = cloud_point.z / numBluePoints;
  cloud_point.r = 255;
  cloud_point.g = 200;
  cloud_point.b = 0;
  
  if(numBluePoints > 0)
  {
    input->points.push_back(cloud_point);
    return true;
  }
  else
  {
    return false;
  }
}

bool
Map_Scan::YellowFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointXYZRGB cloud_point_right;
  cloud_point_right.x = 0.0;
  cloud_point_right.y = 0.0;
  cloud_point_right.z = 0.0;
  pcl::PointXYZRGB cloud_point_left;
  cloud_point_left.x = 0.0;
  cloud_point_left.y = 0.0;
  cloud_point_left.z = 0.0;
  
  int numYellowPoints = 0;
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 255 && input->points[point_id].r == 255 && input->points[point_id].b == 0)
    {
	    if(cloud_point_right.y == 0.0)
	    {
        cloud_point_right.x = input->points[point_id].x;
        cloud_point_right.y = input->points[point_id].y;
        cloud_point_right.z = input->points[point_id].z;
        cloud_point_left.x = input->points[point_id].x;
        cloud_point_left.y = input->points[point_id].y;
        cloud_point_left.z = input->points[point_id].z;
      }
      else
      {
	      if(cloud_point_right.y < input->points[point_id].y)
	      {
          cloud_point_right.x = input->points[point_id].x;
          cloud_point_right.y = input->points[point_id].y;
          cloud_point_right.z = input->points[point_id].z;
        }
	      if(cloud_point_left.y > input->points[point_id].y)
	      {
          cloud_point_left.x = input->points[point_id].x;
          cloud_point_left.y = input->points[point_id].y;
          cloud_point_left.z = input->points[point_id].z;
        }
      }
      numYellowPoints++;
    }
  }
  int right_num_points = 1;
  int left_num_points = 1;
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 255 && input->points[point_id].r == 255 && input->points[point_id].b == 0)
    {
      float right_distance = sqrt(pow(cloud_point_right.x - input->points[point_id].x, 2.0) + pow(cloud_point_right.y - input->points[point_id].y, 2.0));
	    if(right_distance < 1.0)
	    {
        cloud_point_right.x += input->points[point_id].x;
        cloud_point_right.y += input->points[point_id].y;
        cloud_point_right.z += input->points[point_id].z;
        right_num_points++;
      }
      float left_distance = sqrt(pow(cloud_point_left.x - input->points[point_id].x, 2.0) + pow(cloud_point_left.y - input->points[point_id].y, 2.0));
	    if(left_distance < 1.0)
	    {
        cloud_point_left.x += input->points[point_id].x;
        cloud_point_left.y += input->points[point_id].y;
        cloud_point_left.z += input->points[point_id].z;
        left_num_points++;
      }
    }
  }
  
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 255 && input->points[point_id].r == 255 && input->points[point_id].b == 0)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
  cloud_point_right.x = cloud_point_right.x/right_num_points;
  cloud_point_right.y = cloud_point_right.y/right_num_points;
  cloud_point_right.z = cloud_point_right.z/right_num_points;
  cloud_point_right.r = 100;
  cloud_point_right.g = 255;
  cloud_point_right.b = 0;
  cloud_point_left.x = cloud_point_left.x/left_num_points;
  cloud_point_left.y = cloud_point_left.y/left_num_points;
  cloud_point_left.z = cloud_point_left.z/left_num_points;
  cloud_point_left.r = 255;
  cloud_point_left.g = 100;
  cloud_point_left.b = 0;
  
  input->points.push_back(cloud_point_right);
  input->points.push_back(cloud_point_left);
  
  cloud_point_right.x = (cloud_point_left.x + cloud_point_right.x) / 2;
  cloud_point_right.y = (cloud_point_left.y + cloud_point_right.y) / 2;
  cloud_point_right.z = (cloud_point_left.z + cloud_point_right.z) / 2;
  cloud_point_right.r = 255;
  cloud_point_right.g = 255;
  cloud_point_right.b = 0;
  
  if(numYellowPoints > 0)
  {
    input->points.push_back(cloud_point_right);
    return true;
  }
  else
  {
    return false;
  }
}

void
Map_Scan::YellowRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 255 && input->points[point_id].r == 100 && input->points[point_id].b == 0)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
    if(input->points[point_id].g == 100 && input->points[point_id].r == 255 && input->points[point_id].b == 0)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
    if(input->points[point_id].g == 255 && input->points[point_id].r == 255 && input->points[point_id].b == 0)
    {
      input->points.erase(input->points.begin()+point_id);
      point_id--;
    }
  }
}

void
Map_Scan::floorFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output)
{
  tf2_ros::TransformListener tfListener(*tfBuffer);
  geometry_msgs::TransformStamped camera = tfBuffer->lookupTransform(WORLD, CAMERA_FRAME, ros::Time(static_cast<double>(input->header.stamp)/1000000.0));
  geometry_msgs::TransformStamped right_foot_sole = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_SOLE, ros::Time(0), ros::Duration(0.8));
  // Point Cloud for map
  output->header = input->header;
  output->header.frame_id = WORLD;
  this->raw_copy_scan.points.clear();
  this->raw_copy_scan.header = input->header;
  this->raw_copy_scan.header.frame_id = WORLD;
  
  std::vector<float> visual_quaternion_raw(4,0);
  visual_quaternion_raw[0] = camera.transform.rotation.x;
  visual_quaternion_raw[1] = camera.transform.rotation.y;
  visual_quaternion_raw[2] = camera.transform.rotation.z;
  visual_quaternion_raw[3] = camera.transform.rotation.w;
  tf2::Quaternion visual_quaternion(visual_quaternion_raw[0],visual_quaternion_raw[1],visual_quaternion_raw[2],visual_quaternion_raw[3]);
  tf2::Matrix3x3 visual_rotation(visual_quaternion);
  tf2::Matrix3x3 visual_rotation_inverse(visual_rotation.inverse());
  
  int pcl_cloud_size = input->points.size();
  for(int point_id = 0; point_id < pcl_cloud_size; point_id++)
  {
    std::vector<float> temp_pos(3, 0);
    // Nan Location Filter
    if(isnan(input->points[point_id].x) || isnan(input->points[point_id].y))
    {
      input->points[point_id].r = NAN;
      input->points[point_id].g = NAN;
      input->points[point_id].b = NAN;
    }
    else
    {
      temp_pos[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      
      if(temp_pos[2] + camera.transform.translation.z > right_foot_sole.transform.translation.z + 0.15 && temp_pos[2] + camera.transform.translation.z < right_foot_sole.transform.translation.z + 1.5)
      {
        std::vector<float> temp_pos_height(3, 0);
        input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
        input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
        input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
        temp_pos_height[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
        temp_pos_height[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
        temp_pos_height[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
        
        pcl::PointXYZRGB temp_point;
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = 0.5;
        temp_point.r = input->points[point_id].r;
        temp_point.g = input->points[point_id].g;
        temp_point.b = input->points[point_id].b;
        this->raw_copy_scan.points.push_back(temp_point);
      }
      
      // Non Floor Filter
      if(input->points[point_id].r == input->points[point_id].g && input->points[point_id].g == input->points[point_id].b && input->points[point_id].b < 50)
      {
        if(temp_pos[2] + camera.transform.translation.z < right_foot_sole.transform.translation.z+0.02 && temp_pos[2] + camera.transform.translation.z > right_foot_sole.transform.translation.z-0.02)
        {
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
    }
    if(temp_pos[2] > -camera.transform.translation.z - 5.0+0.01 || temp_pos[2] < -camera.transform.translation.z - 5.0-0.01)
    {
      input->points[point_id].x = NAN;
      input->points[point_id].y = NAN;
      input->points[point_id].z = NAN;
    }

    // Nan Location Filter
    std::vector<float> temp_pos_height(3, 0);
    if(!isnan(input->points[point_id].x))
    {
      temp_pos_height[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos_height[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos_height[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      
      pcl::PointXYZRGB temp_point;
      temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
      temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
      temp_point.z = right_foot_sole.transform.translation.z;
      temp_point.r = 30;
      temp_point.g = 30;
      temp_point.b = 30;
      output->points.push_back(temp_point);
    }
  }
}

void
Map_Scan::floorHabFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output)
{
  tf2_ros::TransformListener tfListener(*tfBuffer);
  geometry_msgs::TransformStamped camera = tfBuffer->lookupTransform(WORLD, CAMERA_FRAME, ros::Time(static_cast<double>(input->header.stamp)/1000000.0));
  geometry_msgs::TransformStamped right_foot_sole = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_SOLE, ros::Time(0), ros::Duration(0.8));
  // Point Cloud for map
  output->header = input->header;
  output->header.frame_id = WORLD;
  this->raw_copy_scan.points.clear();
  this->raw_copy_scan.header = input->header;
  this->raw_copy_scan.header.frame_id = WORLD;
  
  std::vector<float> visual_quaternion_raw(4,0);
  visual_quaternion_raw[0] = camera.transform.rotation.x;
  visual_quaternion_raw[1] = camera.transform.rotation.y;
  visual_quaternion_raw[2] = camera.transform.rotation.z;
  visual_quaternion_raw[3] = camera.transform.rotation.w;
  tf2::Quaternion visual_quaternion(visual_quaternion_raw[0],visual_quaternion_raw[1],visual_quaternion_raw[2],visual_quaternion_raw[3]);
  tf2::Matrix3x3 visual_rotation(visual_quaternion);
  tf2::Matrix3x3 visual_rotation_inverse(visual_rotation.inverse());
  
  int pcl_cloud_size = input->points.size();
  for(int point_id = 0; point_id < pcl_cloud_size; point_id++)
  {
    std::vector<float> temp_pos(3, 0);
    // Nan Location Filter
    if(isnan(input->points[point_id].x) || isnan(input->points[point_id].y))
    {
      input->points[point_id].r = NAN;
      input->points[point_id].g = NAN;
      input->points[point_id].b = NAN;
    }
    else
    {
      temp_pos[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      
      if(temp_pos[2] + camera.transform.translation.z > right_foot_sole.transform.translation.z + 0.15 && temp_pos[2] + camera.transform.translation.z < right_foot_sole.transform.translation.z + 1.5)
      {
        std::vector<float> temp_pos_height(3, 0);
        input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
        input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
        input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
        temp_pos_height[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
        temp_pos_height[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
        temp_pos_height[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
        
        pcl::PointXYZRGB temp_point;
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = 0.5;
        temp_point.r = input->points[point_id].r;
        temp_point.g = input->points[point_id].g;
        temp_point.b = input->points[point_id].b;
        this->raw_copy_scan.points.push_back(temp_point);
      }
    }
    if(temp_pos[2] > -camera.transform.translation.z - 5.0+0.01 || temp_pos[2] < -camera.transform.translation.z - 5.0-0.01)
    {
      input->points[point_id].x = NAN;
      input->points[point_id].y = NAN;
      input->points[point_id].z = NAN;
    }
  }
}

void
Map_Scan::overHangFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  for(int point_id = 0; point_id < input->points.size(); point_id++)
  {
    for(int over_hang_point_id = 0; over_hang_point_id < this->raw_copy_scan.points.size(); over_hang_point_id++)
    {
      float distance_comparison = sqrt(pow(this->raw_copy_scan.points[over_hang_point_id].x - input->points[point_id].x, 2.0) + pow(this->raw_copy_scan.points[over_hang_point_id].y - input->points[point_id].y, 2.0));
      if(distance_comparison < 0.16)
      {
        input->points.erase(input->points.begin() + point_id);
        point_id -= 1;
      }
    }
  }
}

void
Map_Scan::overHangHabFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  for(int point_id = 0; point_id < input->points.size(); point_id++)
  {
    for(int over_hang_point_id = 0; over_hang_point_id < this->raw_copy_scan.points.size(); over_hang_point_id++)
    {
      float distance_comparison = sqrt(pow(this->raw_copy_scan.points[over_hang_point_id].x - input->points[point_id].x, 2.0) + pow(this->raw_copy_scan.points[over_hang_point_id].y - input->points[point_id].y, 2.0));
      if(distance_comparison < 0.25)
      {
        input->points.erase(input->points.begin() + point_id);
        point_id -= 1;
      }
    }
  }
}

void
Map_Scan::overHangBlueFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointXYZRGB cloud_point;
  cloud_point.x = 0.0;
  cloud_point.y = 0.0;
  cloud_point.z = 0.0;
  
  int numBluePoints = 0;
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if(input->points[point_id].g == 0 && input->points[point_id].b == 200 && input->points[point_id].r == 0)
    {
      cloud_point.x += input->points[point_id].x;
      cloud_point.y += input->points[point_id].y;
      cloud_point.z += input->points[point_id].z;
      numBluePoints++;
    }
  }
  cloud_point.x = cloud_point.x / numBluePoints;
  cloud_point.y = cloud_point.y / numBluePoints;
  cloud_point.z = cloud_point.z / numBluePoints;
  cloud_point.r = 0;
  cloud_point.g = 0;
  cloud_point.b = 200;
  
  for(int point_id = 0; point_id < input->points.size(); point_id++)
  {
    float distance_comparison = sqrt(pow(cloud_point.x - input->points[point_id].x, 2.0) + pow(cloud_point.y - input->points[point_id].y, 2.0));
    if(distance_comparison < 0.50 && cloud_point.b != input->points[point_id].b && 200 != input->points[point_id].r)
    {
      input->points.erase(input->points.begin() + point_id);
      point_id -= 1;
    }
  }
}

void
Map_Scan::overHangTaskThreeFilter(pcl::PointCloud<pcl::PointXYZRGB>* input)
{
  pcl::PointXYZRGB cloud_point;
  cloud_point.x = 0.0;
  cloud_point.y = 0.0;
  cloud_point.z = 0.0;
  
  int numPoints = 0;
  for(unsigned long point_id = 0; point_id < input->points.size(); point_id++)
  {
    if((input->points[point_id].g == 100 && input->points[point_id].b == 200 && input->points[point_id].r == 0) && (input->points[point_id].g == 200 && input->points[point_id].b == 200 && input->points[point_id].r == 50))
    {
      cloud_point.x += input->points[point_id].x;
      cloud_point.y += input->points[point_id].y;
      cloud_point.z += input->points[point_id].z;
      numPoints++;
    }
  }
  cloud_point.x = cloud_point.x / numPoints;
  cloud_point.y = cloud_point.y / numPoints;
  cloud_point.z = cloud_point.z / numPoints;
  cloud_point.r = 0;
  cloud_point.g = 0;
  cloud_point.b = 200;
  
  for(int point_id = 0; point_id < input->points.size(); point_id++)
  {
    float distance_comparison = sqrt(pow(cloud_point.x - input->points[point_id].x, 2.0) + pow(cloud_point.y - input->points[point_id].y, 2.0));
    if(distance_comparison < 0.750 && !((input->points[point_id].g == 100 && input->points[point_id].b == 200 && input->points[point_id].r == 0) && (input->points[point_id].g == 200 && input->points[point_id].b == 200 && input->points[point_id].r == 50)))
    {
      input->points.erase(input->points.begin() + point_id);
      point_id -= 1;
    }
  }
}

void
Map_Scan::taskOneFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output)
{
  tf2_ros::TransformListener tfListener(*tfBuffer);
  geometry_msgs::TransformStamped camera = tfBuffer->lookupTransform(WORLD, CAMERA_FRAME, ros::Time(static_cast<double>(input->header.stamp)/1000000.0));
  geometry_msgs::TransformStamped right_foot_sole = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_SOLE, ros::Time(0), ros::Duration(0.8));
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer->lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
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
  
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  // Point Cloud for map
  output->header = input->header;
  output->header.frame_id = WORLD;
  
  std::vector<float> visual_quaternion_raw(4,0);
  visual_quaternion_raw[0] = camera.transform.rotation.x;
  visual_quaternion_raw[1] = camera.transform.rotation.y;
  visual_quaternion_raw[2] = camera.transform.rotation.z;
  visual_quaternion_raw[3] = camera.transform.rotation.w;
  tf2::Quaternion visual_quaternion(visual_quaternion_raw[0],visual_quaternion_raw[1],visual_quaternion_raw[2],visual_quaternion_raw[3]);
  tf2::Matrix3x3 visual_rotation(visual_quaternion);
  tf2::Matrix3x3 visual_rotation_inverse(visual_rotation.inverse());
  
  int pcl_cloud_size = input->points.size();
  for(int point_id = 0; point_id < pcl_cloud_size; point_id++)
  {
    std::vector<float> temp_pos(3, 0);
    // Nan Location Filter
    if(isnan(input->points[point_id].x) || isnan(input->points[point_id].y))
    {
      input->points[point_id].r = NAN;
      input->points[point_id].g = NAN;
      input->points[point_id].b = NAN;
    }
    else
    {
      temp_pos[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      float near_zone = sqrt(pow(fabs(center_point.x - (temp_pos[0]+camera.transform.translation.x)), 2.0) + pow(fabs(center_point.y - (temp_pos[1]+camera.transform.translation.y)), 2.0));
      
      // Non Floor Filter
      if(near_zone > 4.0)
      {
        input->points[point_id].x = NAN;
        input->points[point_id].y = NAN;
        input->points[point_id].z = NAN;
        input->points[point_id].r = NAN;
        input->points[point_id].g = NAN;
        input->points[point_id].b = NAN;
      }
      // Non Floor Filter
      else if(input->points[point_id].b == input->points[point_id].g && input->points[point_id].r > 100 && input->points[point_id].b < 50)
      {
        if(temp_pos[2] < -camera.transform.translation.z+1.0 && temp_pos[2] > -camera.transform.translation.z-0.5)
        {
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
      if(input->points[point_id].g < 20 && input->points[point_id].b > 100 && input->points[point_id].r < 20)
      {
        if(temp_pos[2] < -camera.transform.translation.z+1.0 && temp_pos[2] > -camera.transform.translation.z-0.5)
        {
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
    }
    if(temp_pos[2] > -camera.transform.translation.z - 5.0+0.01 || temp_pos[2] < -camera.transform.translation.z - 5.0-0.01)
    {
      input->points[point_id].x = NAN;
      input->points[point_id].y = NAN;
      input->points[point_id].z = NAN;
    }

    // Nan Location Filter
    std::vector<float> temp_pos_height(3, 0);
    if(!isnan(input->points[point_id].x))
    {
      temp_pos_height[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos_height[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos_height[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      
      pcl::PointXYZRGB temp_point;
      temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
      temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
      temp_point.z = 0.0;
      temp_point.r = input->points[point_id].r;
      temp_point.g = input->points[point_id].g;
      temp_point.b = input->points[point_id].b;
      if(input->points[point_id].g < 20 && input->points[point_id].b > 100 && input->points[point_id].r < 20)
      {
        temp_point.r = 0;
        temp_point.g = 0;
        temp_point.b = 200;
      }
      output->points.push_back(temp_point);
    }
  }
}

void
Map_Scan::taskTwoFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output)
{
  tf2_ros::TransformListener tfListener(*tfBuffer);
  geometry_msgs::TransformStamped camera = tfBuffer->lookupTransform(WORLD, CAMERA_FRAME, ros::Time(static_cast<double>(input->header.stamp)/1000000.0));
  geometry_msgs::TransformStamped right_foot_sole = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_SOLE, ros::Time(0), ros::Duration(0.8));
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer->lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
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
  
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  // Point Cloud for map
  output->header = input->header;
  output->header.frame_id = WORLD;
  
  std::vector<float> visual_quaternion_raw(4,0);
  visual_quaternion_raw[0] = camera.transform.rotation.x;
  visual_quaternion_raw[1] = camera.transform.rotation.y;
  visual_quaternion_raw[2] = camera.transform.rotation.z;
  visual_quaternion_raw[3] = camera.transform.rotation.w;
  tf2::Quaternion visual_quaternion(visual_quaternion_raw[0],visual_quaternion_raw[1],visual_quaternion_raw[2],visual_quaternion_raw[3]);
  tf2::Matrix3x3 visual_rotation(visual_quaternion);
  tf2::Matrix3x3 visual_rotation_inverse(visual_rotation.inverse());
  
  float cable_height = 0.0;
  
  int pcl_cloud_size = input->points.size();
  for(int point_id = 0; point_id < pcl_cloud_size; point_id++)
  {
    std::vector<float> temp_pos(3, 0);
    // Nan Location Filter && distance Filter
    if(isnan(input->points[point_id].x) || isnan(input->points[point_id].y) || isnan(input->points[point_id].z))
    {
      input->points[point_id].x = NAN;
      input->points[point_id].y = NAN;
      input->points[point_id].z = NAN;
      input->points[point_id].r = NAN;
      input->points[point_id].g = NAN;
      input->points[point_id].b = NAN;
    }
    else
    {
      temp_pos[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      float near_zone = sqrt(pow(fabs(center_point.x - (temp_pos[0]+camera.transform.translation.x)), 2.0) + pow(fabs(center_point.y - (temp_pos[1]+camera.transform.translation.y)), 2.0));
      
      // Non Floor Filter
      if(near_zone > 4.0)
      {
        input->points[point_id].x = NAN;
        input->points[point_id].y = NAN;
        input->points[point_id].z = NAN;
        input->points[point_id].r = NAN;
        input->points[point_id].g = NAN;
        input->points[point_id].b = NAN;
      }
      if(input->points[point_id].g < 50 && input->points[point_id].r > 100 && input->points[point_id].b < 50)
      {
        if(temp_pos[2]+camera.transform.translation.z < 2.5+right_foot_sole.transform.translation.z && temp_pos[2]+camera.transform.translation.z > 0.75+right_foot_sole.transform.translation.z)
        {
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
      if(input->points[point_id].g < 50 && input->points[point_id].b > 100 && input->points[point_id].r < 50)
      {
        ROS_INFO("Point distance: %f", near_zone);
        if(temp_pos[2]+camera.transform.translation.z < 2.5+right_foot_sole.transform.translation.z && temp_pos[2]+camera.transform.translation.z > 0.75+right_foot_sole.transform.translation.z)
        {
          cable_height = temp_pos[2]+camera.transform.translation.z;
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
      if(input->points[point_id].g > 40 && input->points[point_id].r > 40 && input->points[point_id].b < 20)
      {
        if(temp_pos[2]+camera.transform.translation.z < 2.5+right_foot_sole.transform.translation.z && temp_pos[2]+camera.transform.translation.z > 0.70+right_foot_sole.transform.translation.z)
        {
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
          input->points[point_id].r = 255;
          input->points[point_id].g = 200;
          input->points[point_id].b = 0;
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
      if(input->points[point_id].g > 40 && input->points[point_id].r > 40 && input->points[point_id].g < 60 && input->points[point_id].r < 60 && input->points[point_id].b < 20)
      {
        if(temp_pos[2]+camera.transform.translation.z < 0.3+right_foot_sole.transform.translation.z && temp_pos[2]+camera.transform.translation.z > 0.1+right_foot_sole.transform.translation.z)
        {
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
          input->points[point_id].r = 255;
          input->points[point_id].g = 255;
          input->points[point_id].b = 0;
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
    }
    if(temp_pos[2] > -camera.transform.translation.z - 5.0+0.01 || temp_pos[2] < -camera.transform.translation.z - 5.0-0.01)
    {
      input->points[point_id].x = NAN;
      input->points[point_id].y = NAN;
      input->points[point_id].z = NAN;
    }

    // Nan Location Filter
    std::vector<float> temp_pos_height(3, 0);
    if(!isnan(input->points[point_id].x) || !isnan(input->points[point_id].y) || !isnan(input->points[point_id].z))
    {
      temp_pos_height[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos_height[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos_height[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      
      pcl::PointXYZRGB temp_point;
      if(input->points[point_id].g < 50 && input->points[point_id].b > 100 && input->points[point_id].r < 50)
      {
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = cable_height;
        temp_point.r = 0;
        temp_point.g = 0;
        temp_point.b = 200;
      }
      if(input->points[point_id].g < 50 && input->points[point_id].r > 100 && input->points[point_id].b < 50)
      {
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = right_foot_sole.transform.translation.z + 0.85;
        temp_point.r = input->points[point_id].r;
        temp_point.g = input->points[point_id].g;
        temp_point.b = input->points[point_id].b;
      }
      if(input->points[point_id].g == 200 && input->points[point_id].r == 255 && input->points[point_id].b == 0)
      {
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = right_foot_sole.transform.translation.z + 0.9;
        temp_point.r = input->points[point_id].r;
        temp_point.g = input->points[point_id].g;
        temp_point.b = input->points[point_id].b;
      }
      if(input->points[point_id].g == 255 && input->points[point_id].r == 255 && input->points[point_id].b == 0)
      {
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = right_foot_sole.transform.translation.z + 0.0;
        temp_point.r = input->points[point_id].r;
        temp_point.g = input->points[point_id].g;
        temp_point.b = input->points[point_id].b;
      }
      output->points.push_back(temp_point);
    }
  }
}

void
Map_Scan::taskThreeFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output)
{
  tf2_ros::TransformListener tfListener(*tfBuffer);
  geometry_msgs::TransformStamped camera = tfBuffer->lookupTransform(WORLD, CAMERA_FRAME, ros::Time(static_cast<double>(input->header.stamp)/1000000.0));
  geometry_msgs::TransformStamped right_foot_sole = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_SOLE, ros::Time(0), ros::Duration(0.8));
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer->lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
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
  
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  // Point Cloud for map
  output->header = input->header;
  output->header.frame_id = WORLD;
  
  std::vector<float> visual_quaternion_raw(4,0);
  visual_quaternion_raw[0] = camera.transform.rotation.x;
  visual_quaternion_raw[1] = camera.transform.rotation.y;
  visual_quaternion_raw[2] = camera.transform.rotation.z;
  visual_quaternion_raw[3] = camera.transform.rotation.w;
  tf2::Quaternion visual_quaternion(visual_quaternion_raw[0],visual_quaternion_raw[1],visual_quaternion_raw[2],visual_quaternion_raw[3]);
  tf2::Matrix3x3 visual_rotation(visual_quaternion);
  tf2::Matrix3x3 visual_rotation_inverse(visual_rotation.inverse());
  
  float cable_height = 0.0;
  
  int pcl_cloud_size = input->points.size();
  for(int point_id = 0; point_id < pcl_cloud_size; point_id++)
  {
    std::vector<float> temp_pos(3, 0);
    // Nan Location Filter && distance Filter
    if(isnan(input->points[point_id].x) || isnan(input->points[point_id].y) || isnan(input->points[point_id].z))
    {
      input->points[point_id].x = NAN;
      input->points[point_id].y = NAN;
      input->points[point_id].z = NAN;
      input->points[point_id].r = NAN;
      input->points[point_id].g = NAN;
      input->points[point_id].b = NAN;
    }
    else
    {
      temp_pos[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      float near_zone = sqrt(pow(fabs(center_point.x - (temp_pos[0]+camera.transform.translation.x)), 2.0) + pow(fabs(center_point.y - (temp_pos[1]+camera.transform.translation.y)), 2.0));
      
      // Non Floor Filter
      if(near_zone > 6.0)
      {
        input->points[point_id].x = NAN;
        input->points[point_id].y = NAN;
        input->points[point_id].z = NAN;
        input->points[point_id].r = NAN;
        input->points[point_id].g = NAN;
        input->points[point_id].b = NAN;
      }
      if(input->points[point_id].g > 40 && input->points[point_id].r > 40 && input->points[point_id].g < 200 && input->points[point_id].r < 200 && input->points[point_id].b < 20)
      {
        if(temp_pos[2]+camera.transform.translation.z < 0.3+right_foot_sole.transform.translation.z && temp_pos[2]+camera.transform.translation.z > 0.1+right_foot_sole.transform.translation.z)
        {
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
          input->points[point_id].r = 255;
          input->points[point_id].g = 255;
          input->points[point_id].b = 0;
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
      if(input->points[point_id].r > 20 && input->points[point_id].g > 20 && input->points[point_id].b > 20 && input->points[point_id].r < 50 && input->points[point_id].g < 50 && input->points[point_id].b < 50 && near_zone < 1.0)
      {
        if(temp_pos[2]+camera.transform.translation.z < 2.0+right_foot_sole.transform.translation.z && temp_pos[2]+camera.transform.translation.z > 0.5+right_foot_sole.transform.translation.z)
        {
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
          input->points[point_id].r = 10;
          input->points[point_id].g = 10;
          input->points[point_id].b = 10;
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
      if(input->points[point_id].r > 10 && input->points[point_id].g > 160 && input->points[point_id].b > 160 && input->points[point_id].r < 70 && input->points[point_id].g < 230 && input->points[point_id].b < 230)
      {
        if(temp_pos[2]+camera.transform.translation.z < 2.0+right_foot_sole.transform.translation.z && temp_pos[2]+camera.transform.translation.z > 0.5+right_foot_sole.transform.translation.z)
        {
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
          input->points[point_id].r = 50;
          input->points[point_id].g = 200;
          input->points[point_id].b = 200;
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
      if(input->points[point_id].r < 50 && input->points[point_id].g > 60 && input->points[point_id].b > 180 && input->points[point_id].g < 120 && input->points[point_id].b < 220)
      {
        if(temp_pos[2]+camera.transform.translation.z < 2.0+right_foot_sole.transform.translation.z && temp_pos[2]+camera.transform.translation.z > 0.5+right_foot_sole.transform.translation.z)
        {
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
          input->points[point_id].r = 0;
          input->points[point_id].g = 100;
          input->points[point_id].b = 200;
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
    }
    if(temp_pos[2] > -camera.transform.translation.z - 5.0+0.01 || temp_pos[2] < -camera.transform.translation.z - 5.0-0.01)
    {
      input->points[point_id].x = NAN;
      input->points[point_id].y = NAN;
      input->points[point_id].z = NAN;
    }

    // Nan Location Filter
    std::vector<float> temp_pos_height(3, 0);
    if(!isnan(input->points[point_id].x) || !isnan(input->points[point_id].y) || !isnan(input->points[point_id].z))
    {
      temp_pos_height[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos_height[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos_height[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      
      pcl::PointXYZRGB temp_point;
      if(input->points[point_id].g == 255 && input->points[point_id].r == 255 && input->points[point_id].b == 0)
      {
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = 0.0;
        temp_point.r = input->points[point_id].r;
        temp_point.g = input->points[point_id].g;
        temp_point.b = input->points[point_id].b;
      }
      if(input->points[point_id].g == 10 && input->points[point_id].r == 10 && input->points[point_id].b == 10)
      {
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = 0.0;
        temp_point.r = input->points[point_id].r;
        temp_point.g = input->points[point_id].g;
        temp_point.b = input->points[point_id].b;
      }
      if(input->points[point_id].g == 200 && input->points[point_id].r == 50 && input->points[point_id].b == 200)
      {
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = 0.0;
        temp_point.r = input->points[point_id].r;
        temp_point.g = input->points[point_id].g;
        temp_point.b = input->points[point_id].b;
      }
      if(input->points[point_id].g == 100 && input->points[point_id].r == 0 && input->points[point_id].b == 200)
      {
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = 0.0;
        temp_point.r = input->points[point_id].r;
        temp_point.g = input->points[point_id].g;
        temp_point.b = input->points[point_id].b;
      }
      output->points.push_back(temp_point);
    }
  }
}

void
Map_Scan::taskTwoHandleFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output, std::vector<float> button_point)
{
  tf2_ros::TransformListener tfListener(*tfBuffer);
  geometry_msgs::TransformStamped right_foot_sole = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_SOLE, ros::Time(0), ros::Duration(0.8));
  geometry_msgs::TransformStamped camera = tfBuffer->lookupTransform(WORLD, CAMERA_FRAME, ros::Time(static_cast<double>(input->header.stamp)/1000000.0));
  geometry_msgs::TransformStamped left_foot_ankle = tfBuffer->lookupTransform(WORLD, LEFT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
  geometry_msgs::TransformStamped right_foot_ankle = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_ANKLE, ros::Time(0), ros::Duration(0.2));
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
  
  geometry_msgs::Vector3 center_point = left_foot_ankle.transform.translation;
  center_point.x += right_foot_ankle.transform.translation.x;
  center_point.y += right_foot_ankle.transform.translation.y;
  center_point.z += right_foot_ankle.transform.translation.z;
  
  center_point.x = center_point.x/2;
  center_point.y = center_point.y/2;
  center_point.z = center_point.z/2;
  // Point Cloud for map
  output->header = input->header;
  output->header.frame_id = WORLD;
  ROS_INFO("Start handle scan");
  
  std::vector<float> visual_quaternion_raw(4,0);
  visual_quaternion_raw[0] = camera.transform.rotation.x;
  visual_quaternion_raw[1] = camera.transform.rotation.y;
  visual_quaternion_raw[2] = camera.transform.rotation.z;
  visual_quaternion_raw[3] = camera.transform.rotation.w;
  tf2::Quaternion visual_quaternion(visual_quaternion_raw[0],visual_quaternion_raw[1],visual_quaternion_raw[2],visual_quaternion_raw[3]);
  tf2::Matrix3x3 visual_rotation(visual_quaternion);
  tf2::Matrix3x3 visual_rotation_inverse(visual_rotation.inverse());
  
  int num_handle_points = 0;
  int pcl_cloud_size = input->points.size();
  for(int point_id = 0; point_id < pcl_cloud_size; point_id++)
  {
    std::vector<float> temp_pos(3, 0);
    // Nan Location Filter
    if(isnan(input->points[point_id].x) || isnan(input->points[point_id].y))
    {
      input->points[point_id].x = NAN;
      input->points[point_id].y = NAN;
      input->points[point_id].z = NAN;
      input->points[point_id].r = NAN;
      input->points[point_id].g = NAN;
      input->points[point_id].b = NAN;
    }
    else
    {
      temp_pos[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      
      float distance_comparison = sqrt(pow(button_point[0] - (temp_pos[0]+camera.transform.translation.x), 2.0) + pow(button_point[1] - (temp_pos[1]+camera.transform.translation.y), 2.0));
      // Non Floor Filter
      if(distance_comparison < 0.3 && temp_pos[2]+camera.transform.translation.z > button_point[2] + 0.01 && temp_pos[2]+camera.transform.translation.z < button_point[2] + 0.05)
      {
        temp_pos[2] = -camera.transform.translation.z - 5.0;
        input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
        input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
        input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
        input->points[point_id].r = 0;
        input->points[point_id].g = 0;
        input->points[point_id].b = 200;
        num_handle_points++;
      }
      
      float distance_to_trailer = sqrt(pow(center_point.x - (temp_pos[0]+camera.transform.translation.x), 2.0) + pow(center_point.y - (temp_pos[1]+camera.transform.translation.y), 2.0));
      float heading_to_trailer = asin((center_point.y - (temp_pos[1]+camera.transform.translation.y))/distance_to_trailer);
      if((temp_pos[0]+camera.transform.translation.x) - center_point.x <= 0)
      {
        if((temp_pos[1]+camera.transform.translation.y) - center_point.y <= 0)
        {
          heading_to_trailer = -M_PI-heading_to_trailer;
        }
        else
        {
          heading_to_trailer = M_PI-heading_to_trailer;
        }
      }
      if(distance_comparison < 2.0 && distance_comparison > 1.5 && input->points[point_id].r < 15 && input->points[point_id].g < 15 && input->points[point_id].b < 15 && heading_to_trailer < M_PI/2 - yaw/2 && heading_to_trailer > -M_PI/2 - yaw/2)
      {
        if(temp_pos[2]+camera.transform.translation.z < 1.25+right_foot_sole.transform.translation.z && temp_pos[2]+camera.transform.translation.z > 0.75+right_foot_sole.transform.translation.z)
        {
          temp_pos[2] = -camera.transform.translation.z - 5.0;
          input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
          input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
          input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
          input->points[point_id].r = 10;
          input->points[point_id].g = 10;
          input->points[point_id].b = 10;
        }
        else
        {
          input->points[point_id].x = NAN;
          input->points[point_id].y = NAN;
          input->points[point_id].z = NAN;
        }
      }
    }
    if(temp_pos[2] > -camera.transform.translation.z - 5.0+0.01 || temp_pos[2] < -camera.transform.translation.z - 5.0-0.01)
    {
      input->points[point_id].x = NAN;
      input->points[point_id].y = NAN;
      input->points[point_id].z = NAN;
    }
    // Nan Location Filter
    std::vector<float> temp_pos_height(3, 0);
    if(!isnan(input->points[point_id].x))
    {
      temp_pos_height[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos_height[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos_height[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      
      pcl::PointXYZRGB temp_point;
      if(input->points[point_id].g == 0 && input->points[point_id].r == 0 && input->points[point_id].b == 200)
      {
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = 0.9;
        temp_point.r = input->points[point_id].r;
        temp_point.g = input->points[point_id].g;
        temp_point.b = input->points[point_id].b;
      }
      if(input->points[point_id].g == 10 && input->points[point_id].r == 10 && input->points[point_id].b == 10)
      {
        temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
        temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
        temp_point.z = 0.0;
        temp_point.r = input->points[point_id].r;
        temp_point.g = input->points[point_id].g;
        temp_point.b = input->points[point_id].b;
      }
      output->points.push_back(temp_point);
    }
  }
}

void
Map_Scan::taskTwoArrayFilter(tf2_ros::Buffer* tfBuffer, pcl::PointCloud<pcl::PointXYZRGB>* input, pcl::PointCloud<pcl::PointXYZRGB>* output, std::vector<float> cable_point)
{
  tf2_ros::TransformListener tfListener(*tfBuffer);
  geometry_msgs::TransformStamped camera = tfBuffer->lookupTransform(WORLD, CAMERA_FRAME, ros::Time(static_cast<double>(input->header.stamp)/1000000.0));
  geometry_msgs::TransformStamped right_foot_sole = tfBuffer->lookupTransform(WORLD, RIGHT_FOOT_SOLE, ros::Time(0), ros::Duration(0.8));
  // Point Cloud for map
  output->header = input->header;
  output->header.frame_id = WORLD;
  ROS_INFO("Start array scan");
  
  std::vector<float> visual_quaternion_raw(4,0);
  visual_quaternion_raw[0] = camera.transform.rotation.x;
  visual_quaternion_raw[1] = camera.transform.rotation.y;
  visual_quaternion_raw[2] = camera.transform.rotation.z;
  visual_quaternion_raw[3] = camera.transform.rotation.w;
  tf2::Quaternion visual_quaternion(visual_quaternion_raw[0],visual_quaternion_raw[1],visual_quaternion_raw[2],visual_quaternion_raw[3]);
  tf2::Matrix3x3 visual_rotation(visual_quaternion);
  tf2::Matrix3x3 visual_rotation_inverse(visual_rotation.inverse());
  
  int pcl_cloud_size = input->points.size();
  for(int point_id = 0; point_id < pcl_cloud_size; point_id++)
  {
    std::vector<float> temp_pos(3, 0);
    // Nan Location Filter
    if(isnan(input->points[point_id].x) || isnan(input->points[point_id].y))
    {
      input->points[point_id].r = NAN;
      input->points[point_id].g = NAN;
      input->points[point_id].b = NAN;
    }
    else
    {
      temp_pos[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      
      float distance_comparison = sqrt(pow(cable_point[0] - (temp_pos[0]+camera.transform.translation.x), 2.0) + pow(cable_point[1] - (temp_pos[1]+camera.transform.translation.y), 2.0));
      bool color_compare = input->points[point_id].r == input->points[point_id].g && input->points[point_id].g == input->points[point_id].b && input->points[point_id].b < 30;
      // Non Floor Filter
      if(distance_comparison < 0.65 && color_compare && temp_pos[2]+camera.transform.translation.z < cable_point[2] + 0.1 && temp_pos[2]+camera.transform.translation.z > cable_point[2] - 0.1)
      {
        temp_pos[2] = -camera.transform.translation.z - 5.0;
        input->points[point_id].x = temp_pos[0]*visual_rotation_inverse[0][0] + temp_pos[1]*visual_rotation_inverse[0][1] + temp_pos[2]*visual_rotation_inverse[0][2];
        input->points[point_id].y = temp_pos[0]*visual_rotation_inverse[1][0] + temp_pos[1]*visual_rotation_inverse[1][1] + temp_pos[2]*visual_rotation_inverse[1][2];
        input->points[point_id].z = temp_pos[0]*visual_rotation_inverse[2][0] + temp_pos[1]*visual_rotation_inverse[2][1] + temp_pos[2]*visual_rotation_inverse[2][2];
      }
    }
    if(temp_pos[2] > -camera.transform.translation.z - 5.0+0.01 || temp_pos[2] < -camera.transform.translation.z - 5.0-0.01)
    {
      input->points[point_id].x = NAN;
      input->points[point_id].y = NAN;
      input->points[point_id].z = NAN;
    }

    // Nan Location Filter
    std::vector<float> temp_pos_height(3, 0);
    if(!isnan(input->points[point_id].x))
    {
      temp_pos_height[0] = input->points[point_id].x*visual_rotation[0][0] + input->points[point_id].y*visual_rotation[0][1] + input->points[point_id].z*visual_rotation[0][2];
      temp_pos_height[1] = input->points[point_id].x*visual_rotation[1][0] + input->points[point_id].y*visual_rotation[1][1] + input->points[point_id].z*visual_rotation[1][2];
      temp_pos_height[2] = input->points[point_id].x*visual_rotation[2][0] + input->points[point_id].y*visual_rotation[2][1] + input->points[point_id].z*visual_rotation[2][2];
      
      pcl::PointXYZRGB temp_point;
      temp_point.x = temp_pos_height[0]+camera.transform.translation.x;
      temp_point.y = temp_pos_height[1]+camera.transform.translation.y;
      temp_point.z = right_foot_sole.transform.translation.z + 0.75;
      temp_point.r = 200;
      temp_point.g = 0;
      temp_point.b = 0;
      output->points.push_back(temp_point);
    }
  }
}

void
Map_Scan::leftScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.5;
  direction[1] = 1.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = direction[1];
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.5, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  ros::Duration(3.0).sleep();
  camera_scan_callback.callAvailable(ros::WallDuration());
  
  // Point Cloud for map.
  floorFilter(&tfBuffer, &this->raw_scan, map);
  
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  this->raw_copy_scan = voxelGridFilter(leaf_size, &this->raw_copy_scan);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  this->raw_copy_scan = noiseFilter(leaf_size, &this->raw_copy_scan);
  
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  *map = voxelGridFilter(leaf_size, map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
  overHangFilter(map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::rightScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.5;
  direction[1] = -1.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = direction[1];
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.5, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  ros::Duration(3.0).sleep();
  camera_scan_callback.callAvailable(ros::WallDuration());
  
  // Point Cloud for map.
  floorFilter(&tfBuffer, &this->raw_scan, map);
  
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  this->raw_copy_scan = voxelGridFilter(leaf_size, &this->raw_copy_scan);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  this->raw_copy_scan = noiseFilter(leaf_size, &this->raw_copy_scan);
  
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  *map = voxelGridFilter(leaf_size, map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
  overHangFilter(map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::lowScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.8;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.5, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  ros::Duration(3.0).sleep();
  camera_scan_callback.callAvailable(ros::WallDuration());
  
  // Point Cloud for map.
  floorFilter(&tfBuffer, &this->raw_scan, map);
  
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  this->raw_copy_scan = voxelGridFilter(leaf_size, &this->raw_copy_scan);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  this->raw_copy_scan = noiseFilter(leaf_size, &this->raw_copy_scan);
  
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  *map = voxelGridFilter(leaf_size, map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
  overHangFilter(map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::forwardScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.4;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.5, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  ros::Duration(3.0).sleep();
  camera_scan_callback.callAvailable(ros::WallDuration());
  
  // Point Cloud for map.
  floorFilter(&tfBuffer, &this->raw_scan, map);
  
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  this->raw_copy_scan = voxelGridFilter(leaf_size, &this->raw_copy_scan);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  this->raw_copy_scan = noiseFilter(leaf_size, &this->raw_copy_scan);
  
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  *map = voxelGridFilter(leaf_size, map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
  overHangFilter(map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::wideScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  RedRemovalFilter(map);
  BlueRemovalFilter(map);
  GoldRemovalFilter(map);
  
  leftScan(map);
  rightScan(map);
  lowScan(map);
  forwardScan(map);
  
  overHangFilter(map);
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::leftHabScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.5;
  direction[1] = 0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = direction[1];
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.5, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  ros::Duration(3.0).sleep();
  camera_scan_callback.callAvailable(ros::WallDuration());
  
  // Point Cloud for map.
  floorHabFilter(&tfBuffer, &this->raw_scan, map);
  
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  this->raw_copy_scan = voxelGridFilter(leaf_size, &this->raw_copy_scan);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  this->raw_copy_scan = noiseFilter(leaf_size, &this->raw_copy_scan);
  
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  *map = voxelGridFilter(leaf_size, map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
  overHangHabFilter(map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::rightHabScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.5;
  direction[1] = -0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = direction[1];
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.5, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  ros::Duration(3.0).sleep();
  camera_scan_callback.callAvailable(ros::WallDuration());
  
  // Point Cloud for map.
  floorHabFilter(&tfBuffer, &this->raw_scan, map);
  
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  this->raw_copy_scan = voxelGridFilter(leaf_size, &this->raw_copy_scan);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  this->raw_copy_scan = noiseFilter(leaf_size, &this->raw_copy_scan);
  
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  *map = voxelGridFilter(leaf_size, map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
  overHangHabFilter(map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::lowHabScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -1.2;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.5, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  ros::Duration(3.0).sleep();
  camera_scan_callback.callAvailable(ros::WallDuration());
  
  // Point Cloud for map.
  floorHabFilter(&tfBuffer, &this->raw_scan, map);
  
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  this->raw_copy_scan = voxelGridFilter(leaf_size, &this->raw_copy_scan);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  this->raw_copy_scan = noiseFilter(leaf_size, &this->raw_copy_scan);
  
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  *map = voxelGridFilter(leaf_size, map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
  overHangHabFilter(map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::forwardHabScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.1;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.5, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  ros::Duration(3.0).sleep();
  camera_scan_callback.callAvailable(ros::WallDuration());
  
  // Point Cloud for map.
  floorHabFilter(&tfBuffer, &this->raw_scan, map);
  
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  this->raw_copy_scan = voxelGridFilter(leaf_size, &this->raw_copy_scan);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  this->raw_copy_scan = noiseFilter(leaf_size, &this->raw_copy_scan);
  
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  *map = voxelGridFilter(leaf_size, map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
  overHangHabFilter(map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::wideHabScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  RedRemovalFilter(map);
  BlueRemovalFilter(map);
  GoldRemovalFilter(map);
  
  leftHabScan(map);
  rightHabScan(map);
  lowHabScan(map);
  forwardHabScan(map);
  
  overHangHabFilter(map);
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::narrowScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.8;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = direction[1];
  neck_ang_vel[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg);
  neck_ang_vel[0] = 0.0;
  neck_ang_vel[1] = 0.0;
  neck_ang_vel[2] = 0.0;
  neck_msg = trajectory_generation.appendTrajectoryPoint(3.0, neck_pos, neck_ang_vel, neck_msg);
  direction[0] = -0.4;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[0] = 0.0;
  neck_ang_vel[1] = 0.0;
  neck_ang_vel[2] = 0.0;
  neck_msg = trajectory_generation.appendTrajectoryPoint(4.0, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  for(int num_turns = 0; num_turns < 2; num_turns++)
  {
    ros::Duration(2.625).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
  
    // Point Cloud for map.
    floorFilter(&tfBuffer, &this->raw_scan, map);
  
    std::vector<float> leaf_size(3, 0);
    leaf_size[0] = 0.15;
    leaf_size[1] = 0.15;
    leaf_size[2] = 0.15;
    this->raw_copy_scan = voxelGridFilter(leaf_size, &this->raw_copy_scan);
    leaf_size[0] = 0.35;
    leaf_size[1] = 0.35;
    leaf_size[2] = 0.35;
    this->raw_copy_scan = noiseFilter(leaf_size, &this->raw_copy_scan);
    
    leaf_size[0] = 0.15;
    leaf_size[1] = 0.15;
    leaf_size[2] = 0.15;
    *map = voxelGridFilter(leaf_size, map);
    leaf_size[0] = 0.35;
    leaf_size[1] = 0.35;
    leaf_size[2] = 0.35;
    *map = noiseFilter(leaf_size, map);
  }
  overHangFilter(map);
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::simpleScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Duration(3.0).sleep();
  
  camera_scan_callback.callAvailable(ros::WallDuration());
  
  // Point Cloud for map.
  floorFilter(&tfBuffer, &this->raw_scan, map);
  
  std::vector<float> leaf_size(3, 0);
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  this->raw_copy_scan = voxelGridFilter(leaf_size, &this->raw_copy_scan);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  this->raw_copy_scan = noiseFilter(leaf_size, &this->raw_copy_scan);
  
  leaf_size[0] = 0.15;
  leaf_size[1] = 0.15;
  leaf_size[2] = 0.15;
  *map = voxelGridFilter(leaf_size, map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
  overHangFilter(map);
  leaf_size[0] = 0.35;
  leaf_size[1] = 0.35;
  leaf_size[2] = 0.35;
  *map = noiseFilter(leaf_size, map);
}

void
Map_Scan::simpleTaskOneScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Duration(3.0).sleep();
  
  camera_scan_callback.callAvailable(ros::WallDuration());
  
  // Point Cloud for map.
  taskOneFilter(&tfBuffer, &this->raw_scan, map);
  
  RedFilter(map);
  BlueFilter(map);
}

void
Map_Scan::simpleTaskTwoScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  RedRemovalFilter(map);
  BlueRemovalFilter(map);
  GoldRemovalFilter(map);
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.4;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  ros::Duration(3.0).sleep();
  
  camera_scan_callback.callAvailable(ros::WallDuration());
  
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  RedFilter(map);
}

void
Map_Scan::simpleTaskTwoHandleScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Duration(2.0).sleep();
  
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  RedFilter(map);
  
  ros::Duration(1.5).sleep();
  camera_scan_callback.callAvailable(ros::WallDuration());
  std::vector<float> red_button_point(3, 0);
  // Locate task two panel button
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
    {
      red_button_point[0] = map->points[point_id].x;
      red_button_point[1] = map->points[point_id].y;
      red_button_point[2] = map->points[point_id].z;
    }
  }
  
  // Point Cloud for map.
  taskTwoHandleFilter(&tfBuffer, &this->raw_scan, map, red_button_point);
  BlueFilter(map);
}

void
Map_Scan::wideTaskTwoHandleScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  RedRemovalFilter(map);
  BlueRemovalFilter(map);
  GoldRemovalFilter(map);
  BlackRemovalFilter(map);
  
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(3.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  bool tmp = RedFilter(map);
  bool found_button = tmp;
  if(found_button == true)
  {
    ros::Duration(0.5).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
    std::vector<float> red_button_point(3, 0);
    // Locate task two panel button
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
      {
        red_button_point[0] = map->points[point_id].x;
        red_button_point[1] = map->points[point_id].y;
        red_button_point[2] = map->points[point_id].z;
      }
    }
    // Point Cloud for map.
    taskTwoHandleFilter(&tfBuffer, &this->raw_scan, map, red_button_point);
  }
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_left;
  neck_msg_left.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.5;
  direction[1] = 0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_left = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_left);
  trajectoryNeckPublisher.publish(neck_msg_left);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  tmp = RedFilter(map);
  if(found_button == false)
  {
    found_button = tmp;
  }
  if(found_button == true)
  {
    ros::Duration(0.5).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
    std::vector<float> red_button_point(3, 0);
    // Locate task two panel button
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
      {
        red_button_point[0] = map->points[point_id].x;
        red_button_point[1] = map->points[point_id].y;
        red_button_point[2] = map->points[point_id].z;
      }
    }
    // Point Cloud for map.
    taskTwoHandleFilter(&tfBuffer, &this->raw_scan, map, red_button_point);
  }
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_right;
  neck_msg_right.unique_id = 1;
  direction[0] = -0.5;
  direction[1] = -0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_right = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_right);
  trajectoryNeckPublisher.publish(neck_msg_right);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  tmp = RedFilter(map);
  if(found_button == false)
  {
    found_button = tmp;
  }
  if(found_button == true)
  {
    ros::Duration(0.5).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
    std::vector<float> red_button_point(3, 0);
    // Locate task two panel button
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
      {
        red_button_point[0] = map->points[point_id].x;
        red_button_point[1] = map->points[point_id].y;
        red_button_point[2] = map->points[point_id].z;
      }
    }
    // Point Cloud for map.
    taskTwoHandleFilter(&tfBuffer, &this->raw_scan, map, red_button_point);
  }
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_left_return;
  neck_msg_left_return.unique_id = 1;
  direction[0] = -0.5;
  direction[1] = 0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_left_return = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_left_return);
  trajectoryNeckPublisher.publish(neck_msg_left_return);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  tmp = RedFilter(map);
  if(found_button == false)
  {
    found_button = tmp;
  }
  if(found_button == true)
  {
    ros::Duration(0.5).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
    std::vector<float> red_button_point(3, 0);
    // Locate task two panel button
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
      {
        red_button_point[0] = map->points[point_id].x;
        red_button_point[1] = map->points[point_id].y;
        red_button_point[2] = map->points[point_id].z;
      }
    }
    // Point Cloud for map.
    taskTwoHandleFilter(&tfBuffer, &this->raw_scan, map, red_button_point);
  }
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_straignt;
  neck_msg_straignt.unique_id = 1;
  direction[0] = -0.4;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_straignt = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_straignt);
  trajectoryNeckPublisher.publish(neck_msg_straignt);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  tmp = RedFilter(map);
  if(found_button == false)
  {
    found_button = tmp;
  }
  if(found_button == true)
  {
    ros::Duration(0.5).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
    std::vector<float> red_button_point(3, 0);
    // Locate task two panel button
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
      {
        red_button_point[0] = map->points[point_id].x;
        red_button_point[1] = map->points[point_id].y;
        red_button_point[2] = map->points[point_id].z;
      }
    }
    // Point Cloud for map.
    taskTwoHandleFilter(&tfBuffer, &this->raw_scan, map, red_button_point);
  }
  BlueFilter(map);
  HandleCorrectFilter(map);
  overHangBlueFilter(map);
  BlackFilter(map);
}

void
Map_Scan::wideTaskThreeComponentsScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  RedRemovalFilter(map);
  BlueRemovalFilter(map);
  GoldRemovalFilter(map);
  YellowRemovalFilter(map);
  BlackRemovalFilter(map);
  TealRemovalFilter(map);
  LightBlueRemovalFilter(map);
  
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(2.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_left;
  neck_msg_left.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.5;
  direction[1] = 0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_left = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_left);
  trajectoryNeckPublisher.publish(neck_msg_left);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_right;
  neck_msg_right.unique_id = 1;
  direction[0] = -0.5;
  direction[1] = -0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_right = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_right);
  trajectoryNeckPublisher.publish(neck_msg_right);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_low;
  neck_msg_low.unique_id = 1;
  direction[0] = -0.8;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_low = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_low);
  trajectoryNeckPublisher.publish(neck_msg_low);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_straignt;
  neck_msg_straignt.unique_id = 1;
  direction[0] = -0.1;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_straignt = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_straignt);
  trajectoryNeckPublisher.publish(neck_msg_straignt);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  
  TealFilter(map);
  LightBlueFilter(map);
  overHangTaskThreeFilter(map);
}

void
Map_Scan::simpleTaskThreeHandleScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  RedRemovalFilter(map);
  BlueRemovalFilter(map);
  GoldRemovalFilter(map);
  BlackRemovalFilter(map);
  
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(2.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_straignt;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  neck_msg_straignt.unique_id = 1;
  direction[0] = -0.6;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_straignt = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_straignt);
  trajectoryNeckPublisher.publish(neck_msg_straignt);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  BlackFilter(map);
}

void
Map_Scan::lowTaskTwoHandleScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  RedRemovalFilter(map);
  BlueRemovalFilter(map);
  GoldRemovalFilter(map);
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(2.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -1.2;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  ros::Duration(2.5).sleep();
  
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  RedFilter(map);
  
  ros::Duration(0.5).sleep();
  camera_scan_callback.callAvailable(ros::WallDuration());
  std::vector<float> red_button_point(3, 0);
  // Locate task two panel button
  for(int point_id = 0; point_id < map->points.size(); point_id++)
  {
    if(map->points[point_id].b == 0 && map->points[point_id].g == 0 && map->points[point_id].r == 200)
    {
      red_button_point[0] = map->points[point_id].x;
      red_button_point[1] = map->points[point_id].y;
      red_button_point[2] = map->points[point_id].z;
    }
  }
  
  // Point Cloud for map.
  taskTwoHandleFilter(&tfBuffer, &this->raw_scan, map, red_button_point);
  BlueFilter(map);
  HandleCorrectFilter(map);
}

void
Map_Scan::lowTaskThreeScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(2.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  RedRemovalFilter(map);
  BlueRemovalFilter(map);
  GoldRemovalFilter(map);
  YellowRemovalFilter(map);
  BlackRemovalFilter(map);
  TealRemovalFilter(map);
  LightBlueRemovalFilter(map);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_low;
  neck_msg_low.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.8;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_low = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_low);
  trajectoryNeckPublisher.publish(neck_msg_low);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  TealFilter(map);
  LightBlueFilter(map);
  BlackFilter(map);
}

void
Map_Scan::simpleTaskTwoCableScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(2.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Duration(2.0).sleep();
  
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  bool found_cable = BlueFilter(map);
  if(found_cable == true)
  {
    ros::Duration(1.5).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
    std::vector<float> blue_cable_point(3, 0);
    // Locate task two panel button
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      if(map->points[point_id].b == 200 && map->points[point_id].g == 0 && map->points[point_id].r == 0)
      {
        blue_cable_point[0] = map->points[point_id].x;
        blue_cable_point[1] = map->points[point_id].y;
        blue_cable_point[2] = map->points[point_id].z;
      }
    }
    
    // Point Cloud for map.
    taskTwoArrayFilter(&tfBuffer, &this->raw_scan, map, blue_cable_point);
    RedFilter(map);
  }
}

void
Map_Scan::wideTaskTwoCableScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  RedRemovalFilter(map);
  BlueRemovalFilter(map);
  GoldRemovalFilter(map);
  
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(2.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_left;
  neck_msg_left.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.5;
  direction[1] = 0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_left = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_left);
  trajectoryNeckPublisher.publish(neck_msg_left);
  ros::Duration(2.5).sleep();
  
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  bool found_cable = BlueFilter(map);
  if(found_cable == true)
  {
    ros::Duration(0.5).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
    std::vector<float> blue_cable_point(3, 0);
    // Locate task two panel button
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      if(map->points[point_id].b == 200 && map->points[point_id].g == 0 && map->points[point_id].r == 0)
      {
        blue_cable_point[0] = map->points[point_id].x;
        blue_cable_point[1] = map->points[point_id].y;
        blue_cable_point[2] = map->points[point_id].z;
      }
    }
    
    // Point Cloud for map.
    taskTwoArrayFilter(&tfBuffer, &this->raw_scan, map, blue_cable_point);
  }
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_right;
  neck_msg_right.unique_id = 1;
  direction[0] = -0.5;
  direction[1] = -0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_right = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_right);
  trajectoryNeckPublisher.publish(neck_msg_right);
  ros::Duration(2.5).sleep();
  
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  found_cable = BlueFilter(map);
  if(found_cable == true)
  {
    ros::Duration(0.5).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
    std::vector<float> blue_cable_point(3, 0);
    // Locate task two panel button
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      if(map->points[point_id].b == 200 && map->points[point_id].g == 0 && map->points[point_id].r == 0)
      {
        blue_cable_point[0] = map->points[point_id].x;
        blue_cable_point[1] = map->points[point_id].y;
        blue_cable_point[2] = map->points[point_id].z;
      }
    }
    
    // Point Cloud for map.
    taskTwoArrayFilter(&tfBuffer, &this->raw_scan, map, blue_cable_point);
  }
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_low;
  neck_msg_low.unique_id = 1;
  direction[0] = -0.8;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_low = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_low);
  trajectoryNeckPublisher.publish(neck_msg_low);
  ros::Duration(1.75).sleep();
  
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  found_cable = BlueFilter(map);
  if(found_cable == true)
  {
    ros::Duration(0.5).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
    std::vector<float> blue_cable_point(3, 0);
    // Locate task two panel button
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      if(map->points[point_id].b == 200 && map->points[point_id].g == 0 && map->points[point_id].r == 0)
      {
        blue_cable_point[0] = map->points[point_id].x;
        blue_cable_point[1] = map->points[point_id].y;
        blue_cable_point[2] = map->points[point_id].z;
      }
    }
    
    // Point Cloud for map.
    taskTwoArrayFilter(&tfBuffer, &this->raw_scan, map, blue_cable_point);
  }
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_straignt;
  neck_msg_straignt.unique_id = 1;
  direction[0] = -0.4;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_straignt = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_straignt);
  trajectoryNeckPublisher.publish(neck_msg_straignt);
  ros::Duration(1.0).sleep();
  
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  found_cable = BlueFilter(map);
  if(found_cable == true)
  {
    ros::Duration(0.5).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
    std::vector<float> blue_cable_point(3, 0);
    // Locate task two panel button
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      if(map->points[point_id].b == 200 && map->points[point_id].g == 0 && map->points[point_id].r == 0)
      {
        blue_cable_point[0] = map->points[point_id].x;
        blue_cable_point[1] = map->points[point_id].y;
        blue_cable_point[2] = map->points[point_id].z;
      }
    }
    
    // Point Cloud for map.
    taskTwoArrayFilter(&tfBuffer, &this->raw_scan, map, blue_cable_point);
  }
  RedFilter(map);
  CableCorrectFilter(map);
}

void
Map_Scan::lowTaskTwoCableScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  BlueRemovalFilter(map);
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.8;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = direction[1];
  neck_ang_vel[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(2.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Duration(2.625).sleep();
  
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  bool found_cable = BlueFilter(map);
  if(found_cable == true)
  {
    ros::Duration(0.1).sleep();
    camera_scan_callback.callAvailable(ros::WallDuration());
    std::vector<float> blue_cable_point(3, 0);
    // Locate task two panel button
    for(int point_id = 0; point_id < map->points.size(); point_id++)
    {
      if(map->points[point_id].b == 200 && map->points[point_id].g == 0 && map->points[point_id].r == 0)
      {
        blue_cable_point[0] = map->points[point_id].x;
        blue_cable_point[1] = map->points[point_id].y;
        blue_cable_point[2] = map->points[point_id].z;
      }
    }
    
    // Point Cloud for map.
    taskTwoArrayFilter(&tfBuffer, &this->raw_scan, map, blue_cable_point);
    RedFilter(map);
  }
  CableCorrectFilter(map);
}

void
Map_Scan::lowTaskTwoPlugScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(2.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  BlueRemovalFilter(map);
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg;
  neck_msg.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -1.0;
  direction[1] = -0.3;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_msg = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg);
  trajectoryNeckPublisher.publish(neck_msg);
  
  ros::Duration(2.0).sleep();
  
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskTwoFilter(&tfBuffer, &this->raw_scan, map);
  BlueFilter(map);
  GoldFilter(map);
}

void
Map_Scan::wideTaskThreeStairScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  RedRemovalFilter(map);
  BlueRemovalFilter(map);
  GoldRemovalFilter(map);
  YellowRemovalFilter(map);
  
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(2.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_left;
  neck_msg_left.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -0.5;
  direction[1] = 0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_left = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_left);
  trajectoryNeckPublisher.publish(neck_msg_left);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_right;
  neck_msg_right.unique_id = 1;
  direction[0] = -0.5;
  direction[1] = -0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_right = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_right);
  trajectoryNeckPublisher.publish(neck_msg_right);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_low;
  neck_msg_low.unique_id = 1;
  direction[0] = -0.8;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_low = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_low);
  trajectoryNeckPublisher.publish(neck_msg_low);
  ros::Duration(1.75).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_straignt;
  neck_msg_straignt.unique_id = 1;
  direction[0] = -0.4;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_straignt = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_straignt);
  trajectoryNeckPublisher.publish(neck_msg_straignt);
  ros::Duration(1.0).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  YellowFilter(map);
}

void
Map_Scan::wideLowTaskThreeStairScan(pcl::PointCloud<pcl::PointXYZRGB>* map)
{
  RedRemovalFilter(map);
  BlueRemovalFilter(map);
  GoldRemovalFilter(map);
  YellowRemovalFilter(map);
  
  // Sets up buffer to recieve relative frame positions.
  tf2_ros::Buffer tfBuffer(ros::Duration(2.0), true);
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_left;
  neck_msg_left.unique_id = 1;
  std::vector<float> direction(2, 0);
  std::vector<float> neck_pos(3, 0);
  std::vector<float> neck_ang_vel(3, 0);
  direction[0] = -1.2;
  direction[1] = 0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_left = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_left);
  trajectoryNeckPublisher.publish(neck_msg_left);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_right;
  neck_msg_right.unique_id = 1;
  direction[0] = -1.2;
  direction[1] = -0.8;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_right = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_right);
  trajectoryNeckPublisher.publish(neck_msg_right);
  ros::Duration(2.5).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_low;
  neck_msg_low.unique_id = 1;
  direction[0] = -1.2;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_low = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_low);
  trajectoryNeckPublisher.publish(neck_msg_low);
  ros::Duration(1.75).sleep();
  // Sets up buffer to recieve relative frame positions.
  camera_scan_callback.callAvailable(ros::WallDuration());
  // Point Cloud for map.
  taskThreeFilter(&tfBuffer, &this->raw_scan, map);
  
  ihmc_msgs::NeckTrajectoryRosMessage neck_msg_straignt;
  neck_msg_straignt.unique_id = 1;
  direction[0] = -0.4;
  direction[1] = 0.0;
  neck_pos[0] = 0.431 - direction[0]*0.431/(M_PI/2);
  neck_pos[1] = direction[1];
  neck_pos[2] = -0.431 - direction[0]*0.431/(M_PI/2);
  neck_ang_vel[1] = 0.0;
  neck_msg_straignt = trajectory_generation.appendTrajectoryPoint(1.0, neck_pos, neck_ang_vel, neck_msg_straignt);
  trajectoryNeckPublisher.publish(neck_msg_straignt);
  YellowFilter(map);
}
