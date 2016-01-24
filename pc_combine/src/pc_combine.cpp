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

using namespace std;

ros::Publisher pub;
bool cam1_data_valid = false;
bool cam2_data_valid = false;
bool cam3_data_valid = false;

pcl::PointCloud<pcl::PointXYZ> input1_pcl, input2_pcl, input3_pcl;
pcl::PointCloud<pcl::PointXYZ> output_pcl, output1_pcl, output2_pcl, output3_pcl;

string base_link_id = "base_link";

pcl::PCLPointCloud2 pcl_cam1, pcl_cam2, pcl_combined;
tf::TransformListener *tf_listener;

void pcl_combine ()
{
  cam1_data_valid = false;
  cam2_data_valid = false;
  cam3_data_valid = false;

  sensor_msgs::PointCloud2 output;

  output_pcl = output1_pcl;
  output_pcl += output2_pcl;
  output_pcl += output3_pcl;

  pcl::toROSMsg(output_pcl, output);
  
  pub.publish(output);
}

void cloud_cb_cam1 (const sensor_msgs::PointCloud2ConstPtr& input)
{
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
  pcl_ros::transformPointCloud (input1_pcl, output1_pcl, transform);
  output1_pcl.header.frame_id=base_link_id;

  if (cam1_data_valid && cam2_data_valid && cam3_data_valid)
  {
    pcl_combine();
  }
}

void cloud_cb_cam2 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  cam2_data_valid = true;
  base_link_id = "base_link";

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
  pcl_ros::transformPointCloud (input2_pcl, output2_pcl, transform);
  output2_pcl.header.frame_id=base_link_id;

  if (cam1_data_valid && cam2_data_valid && cam3_data_valid)
  {
	pcl_combine();
  }
}

void cloud_cb_cam3 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  cam3_data_valid = true;
  base_link_id = "base_link";

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
  pcl_ros::transformPointCloud (input3_pcl, output3_pcl, transform);
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

  // Spin
  ros::spin ();
}
