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
#include <std_msgs/Float64.h>

#include "pc_combine.h"

using namespace std;

ros::Publisher pub, new_direction_pub, combine_time_pub, detect_time_pub, avoid_time_pub;
bool cam1_data_valid = false;
bool cam2_data_valid = false;
bool cam3_data_valid = false;

float min_distance = 0.5;
float collision_distance = 0.7;

pcl::PointCloud<pcl::PointXYZ> input1_pcl, input2_pcl, input3_pcl;
pcl::PointCloud<pcl::PointXYZRGB> output_pcl, output1_pcl, output2_pcl, output3_pcl;

string base_link_id = "base_link";

pcl::PCLPointCloud2 pcl_cam1, pcl_cam2, pcl_combined;
tf::TransformListener *tf_listener;
ros::Time oldTime;

void draw_new_direction (float rotation, float distance, bool willCollide)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 21;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::ARROW;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = sin((rotation * M_PI / 180) / 2);
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = distance;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  if (willCollide)
  {
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
  }
  else
  {
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
  }
  marker.color.a = 1.0;
  
  new_direction_pub.publish(marker);
}

bool detect_collision (float degree_theta)
{
  float quadWidth = 0.8;
  float quadHeight = 0.4;
  bool willCollide = false;
  int collidingPoints = 0;
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
      
      if (point.y < quadWidth / 2 && point.y > -quadWidth / 2 && distance < collision_distance && point.z < quadHeight / 2 && point.z > -quadHeight / 2 && distance > min_distance)
      {
	collidingPoints = collidingPoints + 1;
	//std::cout << "Colliding points: " << collidingPoints << " ";
	if (collidingPoints > 20)
	{
	  willCollide = true;
	  break;
	}
      }
      //output_pcl.at(i) = point;
    }
  }
  return willCollide;
}

void avoid_collision () 
{
  int max_collision_angle = 90;
  int value_to_add = 5;
  int i = value_to_add;
  bool willCollide = true;
  for (int degree = 5; degree <= max_collision_angle; degree = degree + i)
  {
    if (detect_collision(degree))
    {
      //std::cout << "Collision detected" << std::endl;
      //draw_new_direction(-degree, 0.5, true);
    }
    else
    {
      //std::cout << "No collision" << std::endl;
      draw_new_direction(-degree, 4.0, false);
      willCollide = false;
      break;
    }
    
    if (detect_collision(-degree))
    {
      //std::cout << "Collision detected" << std::endl;
      //draw_new_direction(-degree, 0.5, true);
    }
    else
    {
      //std::cout << "No collision" << std::endl;
      draw_new_direction(degree, 4.0, false);
      willCollide = false;
      break;
    }
  }
  
  if (willCollide)
  {
    draw_new_direction(0, 0.5, true);
  }
}

void colorize ()
{
  float quadWidth = 0.8;
  float quadHeight = 0.4;
  bool willCollide = false;
  int collidingPoints = 0;
  
  ros::Time start_detect_time = ros::Time::now();

  for (int i = 0; i < output_pcl.size(); i++)
  {
    pcl::PointXYZRGB point = output_pcl.at(i);
    double distance = sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));
    //std::cout << "d= " << distance << std::endl;
    if (distance > 0 || distance < 0 || distance == 0) {
      if (point.y < quadWidth / 2 && point.y > -quadWidth / 2 && distance < collision_distance && point.z < quadHeight / 2 && point.z > -quadHeight / 2 && distance > min_distance)
      {
	point.r = 255;
	point.g = 255;
	point.b = 255;
	
	collidingPoints++;
	if (collidingPoints > 20)
	{
	  willCollide = true;
	}
      }
      else if (distance < min_distance)
      {
	
      }
      else if (distance < collision_distance)
      {
	point.r = 255;
      }
      else if (distance < 2 * collision_distance)
      {
	point.r = 255;
	point.g = 255;
      }
      else
      {
	point.b = 255;
      }
    }
    
    output_pcl.at(i) = point;
  }
  
  std_msgs::Float64 detect_time;
  detect_time.data = (ros::Time::now() - start_detect_time).toSec() * 1000;
  detect_time_pub.publish(detect_time);

  if (willCollide) 
  {
    ros::Time start_avoid_time = ros::Time::now();
    avoid_collision();
    std_msgs::Float64 avoid_time;
    avoid_time.data = (ros::Time::now() - start_avoid_time).toSec() * 1000;
    avoid_time_pub.publish(avoid_time);
  }
  else
  {
    draw_new_direction(0, 4.0, false);
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

  ros::Time start_combine_time = ros::Time::now();
  output_pcl = output1_pcl;
  output_pcl += output2_pcl;
  output_pcl += output3_pcl;
  std_msgs::Float64 combine_time;
  combine_time.data = (ros::Time::now() - start_combine_time).toSec() * 1000;
  combine_time_pub.publish(combine_time);

  colorize();

  pcl::toROSMsg(output_pcl, output);

  pub.publish(output);
  
  newTime = ros::Time::now();
  std::cout << "Time since last point clould published: " << (newTime - oldTime) << std::endl;
  oldTime = newTime;
}

void cloud_cb_cam1 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!cam1_data_valid)
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
}

void cloud_cb_cam2 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!cam2_data_valid)
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
}

void cloud_cb_cam3 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!cam3_data_valid)
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
  combine_time_pub = nh.advertise<std_msgs::Float64>("/pointcloud/combine_time", 1);
  detect_time_pub = nh.advertise<std_msgs::Float64>("/pointcloud/detect_time", 1);
  avoid_time_pub = nh.advertise<std_msgs::Float64>("/pointcloud/avoid_time", 1);
  
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_in", 1);
  new_direction_pub = nh.advertise<visualization_msgs::Marker>("new_direction", 1);
  // Spin
  ros::spin ();
}
