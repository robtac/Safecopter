#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>

// PCL Includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>


#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

using namespace std;
//using namespace octomap;

ros::Publisher pub, new_direction_pub, vis_cube_pub, binary_map_pub, safecopter_pub, combine_time_pub, detect_time_pub, avoid_time_pub, convert_time_pub, ground_time_pub, total_time_pub, will_collide_pub, can_find_path_pub;
bool cam1_data_valid = false;
bool cam2_data_valid = false;
bool cam3_data_valid = false;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> input1_pcl, input2_pcl, input3_pcl;
pcl::PointCloud<pcl::PointXYZRGB> filtered_pcl, output_pcl, output1_pcl, output2_pcl, output3_pcl;
octomap::OcTree *octmap;

string base_link_id = "base_link";

pcl::PCLPointCloud2 pcl_cam1, pcl_cam2, pcl_combined;
tf::TransformListener *tf_listener;
ros::Time oldTime;

void pcl_combine ()
{
  ros::Time start_total_time = ros::Time::now();

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

//  octomap::OcTree tree = octomap::OcTree(0.05);
//  octmap = &tree;

  ros::Time start_convert_time = ros::Time::now();
  octmap = new octomap::OcTree(0.05);
  octmap->setProbHit(0.9);
  octmap->setProbMiss(0.1);
  octmap->setClampingThresMin(0.49);
  octmap->setClampingThresMax(0.51);
  octomap::KeySet occupied_cells;

  bool valid_tf = false;
  tf::StampedTransform transform;
  try
  {
      tf_listener->lookupTransform("odom", base_link_id, ros::Time(0), transform);
      valid_tf = true;
  }
  catch (tf::TransformException e)
  {
      ROS_ERROR_STREAM("Transform to odom failed " << e.what());
  }

  if (valid_tf)
  {
      pcl_ros::transformPointCloud (output_pcl, output_pcl, transform);

      for(pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = output_pcl.begin(); it != output_pcl.end(); it++)
      {
          octomap::point3d point(it->x, it->y, it->z);
          octomap::OcTreeKey key;
          if (octmap->coordToKeyChecked(point, key))
          {
              occupied_cells.insert(key);
          }
      }

      for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++)
      {
          octmap->updateNode(*it, true);
      }

      octomap::point3d *sensor_origin = new octomath::Vector3(0, 0, 0);


      octmap->insertPointCloud(, sensor_origin);
  }

//  pcl::toROSMsg(output_pcl, output);

//  pub.publish(output);

//  std_msgs::Float64 total_time;
//  total_time.data = (ros::Time::now() - start_total_time).toSec() * 1000;
//  total_time_pub.publish(total_time);

  delete octmap;
}

void cloud_cb_cam1 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!cam1_data_valid)
  {
//    std::cout << "Cam 1; ";
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
//    std::cout << "Cam 2; ";
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
//    std::cout << "Cam 3; ";
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

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "safecopter");
    ros::NodeHandle nh ("safecopter");

    tf_listener = new tf::TransformListener();

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/pf_FF_dephcam/points", 1, cloud_cb_cam1);
    ros::Subscriber sub3 = nh.subscribe ("/pf_FR_dephcam/points", 1, cloud_cb_cam2);
    ros::Subscriber sub2 = nh.subscribe ("/pf_FL_dephcam/points", 1, cloud_cb_cam3);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_in", 1);

    binary_map_pub = nh.advertise<octomap_msgs::Octomap>("octomap_binary", 1, m_latchedTopics);

    // Spin
    ros::spin ();

}
