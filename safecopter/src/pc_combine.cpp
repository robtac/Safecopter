#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/Marker.h>

#include <math.h>

#include "pc_combine.h"

using namespace std;

ros::Publisher pub, pub2;
bool cam1_data_valid = false;
bool cam2_data_valid = false;
bool cam3_data_valid = false;

pcl::PointCloud<pcl::PointXYZ> input1_pcl, input2_pcl, input3_pcl;
pcl::PointCloud<pcl::PointXYZRGB> output_pcl, output1_pcl, output2_pcl, output3_pcl;

string base_link_id = "base_link";

pcl::PCLPointCloud2 pcl_cam1, pcl_cam2, pcl_combined;
tf::TransformListener *tf_listener;
ros::Time oldTime;

bool detect_collision (float degree_theta)
{
  float quadWidth = 0.8;
  float quadHeight = 0.4;
  bool willCollide = false;
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  float theta = (M_PI / 180) * degree_theta;
  
  pcl::PointXYZRGB point;
  
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  
  for (int i = 0; i < output_pcl.size(); i++)
  {
    pcl::PointXYZRGB originalPoint = output_pcl.at(i);
    double distance = sqrt((originalPoint.x * originalPoint.x) + (originalPoint.y * originalPoint.y) + (originalPoint.z * originalPoint.z));
    //std::cout << "d= " << distance << std::endl;
    if (distance > 0 || distance < 0 || distance == 0) { // Null test
      point = pcl::transformPoint (originalPoint, transform_2);
      //std::cout << "OriginalPoint: " << originalPoint.x << " " << originalPoint.y << " " << originalPoint.z << " New Point: " << point.x << " " << point.y << " " << point.z << std::endl;
      
      if (point.x < quadWidth / 2 && point.x > -quadWidth / 2 && distance < 2 && point.z < quadHeight / 2 && point.z > -quadHeight / 2)
      {
	willCollide = true;
	break;
      }
      output_pcl.at(i) = point;
    }
  }
  return willCollide;
}

void avoid_collision () 
{
  bool willCollide = detect_collision(30);
  
  std::cout << "detect_collision: " << willCollide << std::endl;
}

void colorize ()
{
  float quadWidth = 0.8;
  float quadHeight = 0.4;
  bool willCollide = false;
  
  for (int i = 0; i < output_pcl.size(); i++)
  {
    pcl::PointXYZRGB point = output_pcl.at(i);
    double distance = sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));
    //std::cout << "d= " << distance << std::endl;
    if (distance > 0 || distance < 0 || distance == 0) {
      if (point.x < quadWidth / 2 && point.x > -quadWidth / 2 && distance < 2 && point.z < quadHeight / 2 && point.z > -quadHeight / 2)
      {
	point.r = 255;
	point.g = 255;
	point.b = 255;
	
	willCollide = true;
      }
      else if (distance < 1.0)
      {
	point.r = 255;
      }
      else if (distance < 2.0)
      {
	point.g = 255;
      }
      else
      {
	point.b = 255;
      }
    }
    
    output_pcl.at(i) = point;
  }
  
  if (willCollide) 
  {
    avoid_collision();
  }
  else
  {
    std::cout << "Nothing to avoid" << std::endl;
  }
}

void pcl_combine ()
{
  ros::Time newTime;
  
  cam1_data_valid = false;
  cam2_data_valid = false;
  cam3_data_valid = false;

  sensor_msgs::PointCloud2 output;

  output_pcl = output1_pcl;
  output_pcl += output2_pcl;
  output_pcl += output3_pcl;

  colorize();

  pcl::toROSMsg(output_pcl, output);

  pub.publish(output);
  
  newTime = ros::Time::now();
  std::cout << "Time since last point clould published: " << (newTime - oldTime) << std::endl;
  oldTime = newTime;
}

void cloud_cb_cam1 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  std::cout << "Cam 1; ";
  cam1_data_valid = true;
  
  pcl::fromROSMsg(*input, input1_pcl);
  tf::StampedTransform transform;
  try
  {
	  tf_listener->lookupTransform(base_link_id, input->header.frame_id, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
	  ROS_ERROR("%s", ex.what());
  }

  copyPointCloud(input1_pcl, output1_pcl);
  pcl_ros::transformPointCloud (output1_pcl, output1_pcl, transform);
  output1_pcl.header.frame_id=base_link_id;

  if (cam1_data_valid && cam2_data_valid && cam3_data_valid)
  {
    pcl_combine();
  }
}

void cloud_cb_cam2 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  std::cout << "Cam 2; ";
  cam2_data_valid = true;

  pcl::fromROSMsg(*input, input2_pcl);
  tf::StampedTransform transform;
  try
  {
	  tf_listener->lookupTransform(base_link_id, input->header.frame_id, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
	  ROS_ERROR("%s", ex.what());
  }

  copyPointCloud(input2_pcl, output2_pcl);
  pcl_ros::transformPointCloud (output2_pcl, output2_pcl, transform);
  output2_pcl.header.frame_id=base_link_id;

  if (cam1_data_valid && cam2_data_valid && cam3_data_valid)
  {
	pcl_combine();
  }
}

void cloud_cb_cam3 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  std::cout << "Cam 3; ";
  cam3_data_valid = true;

  pcl::fromROSMsg(*input, input3_pcl);
  tf::StampedTransform transform;
  try
  {
	  tf_listener->lookupTransform(base_link_id, input->header.frame_id, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
	  ROS_ERROR("%s", ex.what());
  }

  copyPointCloud(input3_pcl, output3_pcl);
  pcl_ros::transformPointCloud (output3_pcl, output3_pcl, transform);
  output3_pcl.header.frame_id=base_link_id;

  if (cam1_data_valid && cam2_data_valid && cam3_data_valid)
  {
    pcl_combine();
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc_combine");
  ros::NodeHandle nh;

  tf_listener = new tf::TransformListener();

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/pf1/points", 1, cloud_cb_cam1);
  ros::Subscriber sub2 = nh.subscribe ("/pf2/points", 1, cloud_cb_cam2);
  ros::Subscriber sub3 = nh.subscribe("/pf3/points", 1, cloud_cb_cam3);
  
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("", 1);

  // Spin
  ros::spin ();
}
