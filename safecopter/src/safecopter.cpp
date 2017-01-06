#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>

// PCL Includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

// FCL Includes
#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"
#include "fcl/collision_data.h"
#include "fcl/collision_object.h"
#include "fcl/collision.h"
#include "fcl/continuous_collision.h"
#include "fcl/distance.h"

#include <visualization_msgs/Marker.h>

#include <math.h>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "safecopter.h"

using namespace std;

ros::Publisher pub, new_direction_pub, visCubePub;
bool cam1_data_valid = false;
bool cam2_data_valid = false;
bool cam3_data_valid = false;

float min_distance = 0.5f;
float collision_distance = 0.7;
float resolution = 0.01;

octomap::OcTree* octree = new octomap::OcTree(0.05);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> input1_pcl, input2_pcl, input3_pcl;
pcl::PointCloud<pcl::PointXYZRGB> filtered_pcl, output_pcl, output1_pcl, output2_pcl, output3_pcl;

fcl::CollisionObject * collision_box;
fcl::CollisionObject * collision_tree;

string base_link_id = "base_link";

pcl::PCLPointCloud2 pcl_cam1, pcl_cam2, pcl_combined;
tf::TransformListener *tf_listener;
ros::Time oldTime;

void draw_new_direction (float rotation, float distance, bool willCollide)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = base_link_id;
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

void drawCube(fcl::Vec3f vec, int c_color, fcl::Vec3f size, fcl::Matrix3f rotation_mat)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = base_link_id;
        marker.header.stamp = ros::Time();
        marker.ns = "basic_shapes";
        marker.id = 22;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = vec[0];
        marker.pose.position.y = vec[1];
        marker.pose.position.z = vec[2];

        fcl::Quaternion3f rotation_quat;
        rotation_quat.fromRotation(rotation_mat);
        marker.pose.orientation.x = rotation_quat.getX();
        marker.pose.orientation.y = rotation_quat.getY();
        marker.pose.orientation.z = rotation_quat.getZ();
        marker.pose.orientation.w = rotation_quat.getW();

        marker.scale.x = size[0];
        marker.scale.y = size[1];
        marker.scale.z = size[2];
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        if(c_color == 0)
        {
            marker.color.r = 0.0;
            marker.color.b = 0.0;
            marker.color.g = 0.0;
        }
        if(c_color == 1)
        {
            marker.color.r = 0.0;
            marker.color.b = 1.0;
            marker.color.g = 0.0;
        }
        else if(c_color == 2)
        {
            marker.color.r = 1.0;
            marker.color.b = 0.0;
            marker.color.g = 0.0;
        }
        else
        {
            marker.color.r = 0.0;
            marker.color.b = 0.0;
            marker.color.g = 1.0;
        }
        //marker.lifetime = ros::Duration(0.3);
        visCubePub.publish(marker);
}

bool detect_collision (float degree_theta)
{
  float quadWidth = 0.8;
  float quadHeight = 0.4;
  bool willCollide = false;
  float theta = (M_PI / 180) * -degree_theta;
  //fcl::Matrix3f rotation_mat = fcl::Matrix3f(1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta)); // X rotation
  //fcl::Matrix3f rotation_mat = fcl::Matrix3f(cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta)); // Y rotation
  fcl::Matrix3f rotation_mat = fcl::Matrix3f(cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1); // Z rotation

  fcl::Vec3f size = fcl::Vec3f(collision_distance, quadWidth, quadHeight);
  fcl::Box *box = new fcl::Box(size[0], size[1], size[2]);
  box->cost_density = 100;
  box->threshold_occupied = 5;

  //boost::shared_ptr<octomap::OcTree> ptr = octree;
  fcl::OcTree* tree = new fcl::OcTree(boost::shared_ptr<const octomap::OcTree>(octree));

  fcl::Vec3f box_center = fcl::Vec3f(collision_distance / 2 + min_distance, 0, 0);
  fcl::Vec3f translation_vec = fcl::Vec3f((box_center[0] * cos(theta) - box_center[1] * sin(theta)), (box_center[1] * cos(theta) + box_center[0] * sin(theta)), box_center[2]);
  fcl::Transform3f transformation = fcl::Transform3f(rotation_mat, translation_vec);
  collision_box = new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box), transformation);
  collision_tree = new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(tree), fcl::Transform3f());

  fcl::CollisionRequest request;
  fcl::CollisionResult result;

  fcl::collide(collision_box, collision_tree, request, result);

  fcl::AABB b = collision_box -> getAABB();
  fcl::Vec3f vec = b.center();

  if (result.isCollision() == true)
  {
      drawCube(vec, 2, size, rotation_mat);
  } else {
      drawCube(vec, 3, size, rotation_mat);
  }

  if (result.numContacts() > 0)
  {
      willCollide = true;
  }
  return willCollide;
}

void avoid_collision () 
{
  int max_collision_angle = 90;
  int value_to_add = 5;
  bool willCollide = true;
  int counter = 0;
  for (int degree = 0; degree <= max_collision_angle; degree = degree + value_to_add)
  {
      counter++;
    if (detect_collision(degree))
    {
      //std::cout << "Collision detected" << std::endl;
      draw_new_direction(-degree, 0.5, true);
    }
    else
    {
      //std::cout << "No collision" << std::endl;
      draw_new_direction(-degree, 4.0, false);
      willCollide = false;
      break;
    }

    if (degree != 0)
    {
        if (detect_collision(-degree))
        {
          //std::cout << "Collision detected" << std::endl;
          draw_new_direction(-degree, 0.5, true);
        }
        else
        {
          //std::cout << "No collision" << std::endl;
          draw_new_direction(degree, 4.0, false);
          willCollide = false;
          break;
        }
    }
  }
  
  std::cout << "Counter: " << counter;
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
  for (int i = 0; i < output_pcl.size(); i++)
  {
    pcl::PointXYZRGB point = output_pcl.at(i);
    double distance = sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));
    //std::cout << "d= " << distance << std::endl;
    if (distance > 0 || distance < 0 || distance == 0) { // null test
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
        output_pcl.at(i) = point;
      }
      else if (distance < 2 * collision_distance)
      {
        point.r = 255;
        point.g = 255;
        output_pcl.at(i) = point;
      }
      else
      {
        point.b = 255;
        output_pcl.at(i) = point;
      }
    }
  }
}

void callback_tree (const octomap_msgs::Octomap & msg){

//  octomap::AbstractOcTree* tree = new octomap::AbstractOcTree::AbstractOcTree();
//  tree = octomap_msgs::binaryMsgToMap (msg);
  octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap (msg);
  octree = dynamic_cast<octomap::OcTree*>(tree);
  ros::Time firstTime = ros::Time::now();
  avoid_collision();
  std::cout << "Avoid Collision Time: " << ros::Time::now() - firstTime << std::endl;
  delete(tree);
}

void pcl_combine ()
{
  ros::Time newTime;
  ros::Time firstTime = ros::Time::now();
  
  cam1_data_valid = false;
  cam2_data_valid = false;
  cam3_data_valid = false;

  sensor_msgs::PointCloud2 output;

  output_pcl = output1_pcl;
  output_pcl += output2_pcl;
  output_pcl += output3_pcl;

//  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
//  sor.setInputCloud(output_pcl);
//  sor.setMeanK(50);
//  sor.setStddevMulThres(1.0);
//  sor.filter(*filtered_pcl);

  colorize();

  pcl::toROSMsg(output_pcl, output);

  pub.publish(output);
  
  newTime = ros::Time::now();
  //std::cout << "Pcl_combine time: " << (newTime - firstTime) << std::endl;
  //std::cout << "Time since last point clould published: " << (newTime - oldTime) << std::endl;
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
  ros::Subscriber subTree = nh.subscribe("/octomap_binary", 1, &callback_tree);
  
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_in", 1);
  new_direction_pub = nh.advertise<visualization_msgs::Marker>("new_direction", 1);
  visCubePub = nh.advertise<visualization_msgs::Marker>("collision_box", 1);
  // Spin
  ros::spin ();
}
