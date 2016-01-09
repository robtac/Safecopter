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

ros::Publisher pub;
bool cam1_data_valid = false;
bool cam2_data_valid = false;

pcl::PCLPointCloud2 pcl_cam1, pcl_cam2, pcl_combined;
tf::TransformListener* tf_listener = NULL;

void pcl_combine ()
{
  pcl::concatenatePointCloud (pcl_cam1, pcl_cam2, pcl_combined);
    
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(pcl_combined, output);
  
  pub.publish(output);
}

void cloud_cb_cam1 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  cam1_data_valid = true;
  
  // Create a container for the data.
  sensor_msgs::PointCloud2 ros_cam1;

  // Do data processing here...
  pcl_ros::transformPointCloud("pf1_link", *input, ros_cam1, *tf_listener);
  pcl_conversions::toPCL(ros_cam1, pcl_cam1);

  // Publish the data.
  //pub.publish (output);
  if (cam1_data_valid && cam2_data_valid)
  {
    cam1_data_valid = false;
    cam2_data_valid = false;
    
    pcl_combine();
  }
}

void cloud_cb_cam2 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  cam2_data_valid = true;
  
  // Create a container for the data.
  sensor_msgs::PointCloud2 ros_cam2;

  // Do data processing here...
  pcl_ros::transformPointCloud("pf2_link", *input, ros_cam2, *tf_listener);
  pcl_conversions::toPCL(ros_cam2, pcl_cam2);

  // Publish the data.
  //pub.publish (output);
  if (cam1_data_valid && cam2_data_valid)
  {
    cam1_data_valid = false;
    cam2_data_valid = false;
    
    pcl_combine();
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc_combine");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/pf1/points", 1, cloud_cb_cam1);
  ros::Subscriber sub2 = nh.subscribe ("/pf2/points", 1, cloud_cb_cam2);
  
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}