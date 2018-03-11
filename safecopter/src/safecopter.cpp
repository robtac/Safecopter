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

#include "safecopter.h"

using namespace std;

ros::Publisher pub, new_direction_pub, vis_cube_pub, binary_map_pub, safecopter_pub, combine_time_pub, detect_time_pub, avoid_time_pub, convert_time_pub, ground_time_pub, total_time_pub, will_collide_pub, can_find_path_pub;
bool cam1_data_valid = false;
bool cam2_data_valid = false;
bool cam3_data_valid = false;

float resolution = 0.05;

// Parameters
float m_min_distance;               float default_min_distance = 0.5;
float m_collision_distance;         float default_collision_distance = 4.0;
float m_quad_width;                 float default_quad_width = 1.2;
float m_quad_height;                float default_quad_height = 0.4;
float m_ground_minimum;             float default_ground_minimum = 0.5;
bool m_filter_ground;               bool default_filter_ground = false;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> input1_pcl, input2_pcl, input3_pcl;
pcl::PointCloud<pcl::PointXYZRGB> filtered_pcl, output_pcl, output1_pcl, output2_pcl, output3_pcl;
octomap::OcTree *octmap;

fcl::CollisionObject * collision_box;
fcl::CollisionObject * collision_tree;

string base_link_id = "base_link";

pcl::PCLPointCloud2 pcl_cam1, pcl_cam2, pcl_combined;
tf::TransformListener *tf_listener;
ros::Time oldTime;

bool m_latchedTopics;

void print_param_changes () {
    if (m_min_distance != default_min_distance)
    {
        ROS_INFO_STREAM("Changed min_distance from " << default_min_distance << " to " << m_min_distance);
    }
    else
    {
        ROS_INFO_STREAM("min_distance at default " << default_min_distance);
    }

    if (m_collision_distance != default_collision_distance)
    {
        ROS_INFO_STREAM("Changed collision_distance from " << default_collision_distance << " to " << m_collision_distance);
    }
    else
    {
        ROS_INFO_STREAM("collision_distance at default " << default_collision_distance);
    }

    if (m_quad_width != default_quad_width)
    {
        ROS_INFO_STREAM("Changed quad_width from " << default_quad_width << " to " << m_quad_width);
    }
    else
    {
        ROS_INFO_STREAM("quad_width at default " << default_quad_width);
    }

    if (m_quad_height != default_quad_height)
    {
        ROS_INFO_STREAM("Changed quad_height from " << default_quad_height << " to " << m_quad_height);
    }
    else
    {
        ROS_INFO_STREAM("quad_height at default " << default_quad_height);
    }

    if (m_ground_minimum != default_ground_minimum)
    {
        ROS_INFO_STREAM("Changed ground_minimum from " << default_ground_minimum << " to " << m_ground_minimum);
    }
    else
    {
        ROS_INFO_STREAM("ground_minimum at default " << default_ground_minimum);
    }

    if (m_filter_ground != default_filter_ground)
    {
        ROS_INFO_STREAM("Changed filter_ground from " << default_filter_ground << " to " << m_filter_ground);
    }
    else
    {
        ROS_INFO_STREAM("filter_ground at default " << default_filter_ground);
    }
}

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
        vis_cube_pub.publish(marker);
}

void pub_safecopter_data (bool isCollision, bool canFindPath, double direction) {
    std_msgs::Float32 collisionDirection;
    collisionDirection.data = direction;
    safecopter_pub.publish(collisionDirection);

    std_msgs::Bool willCollide;
    willCollide.data = isCollision;
    will_collide_pub.publish(willCollide);

    std_msgs::Bool canFindPath_msg;
    canFindPath_msg.data = canFindPath;
    can_find_path_pub.publish(canFindPath_msg);
}

void detect_object () {
    bool willCollide = false;
    int collidingPoints = 0;
    pcl::PointXYZRGB closestPoint = output_pcl.at(0);
    for (int i = 0; i < output_pcl.size(); i++)
    {
      pcl::PointXYZRGB point = output_pcl.at(i);
      float distance = sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));
      float closestPointDistance = sqrt((closestPoint.x * closestPoint.x) + (closestPoint.y * closestPoint.y) + (closestPoint.z * closestPoint.z));
      if (closestPointDistance > 0 || closestPointDistance < 0 || closestPointDistance == 0)
      {
          if (distance > 0 || distance < 0 || distance == 0) { // null test
            if (distance < closestPointDistance) {
                closestPoint = point;
            }

            if (distance > m_min_distance && distance < m_collision_distance)
            {
                collidingPoints++;
                if (collidingPoints > 20)
                {
                  willCollide = true;
                }
            }
          }
      } else {
          if (distance > 0 || distance < 0 || distance == 0) {
            closestPoint = point;
          }
      }
    }
    float theta = atan(closestPoint.x / closestPoint.y);
    float degreeTheta = theta * 180 / M_PI;
//    pub_safecopter_data(willCollide, degreeTheta);
}

bool detect_collision (float degree_theta)
{
  bool willCollide = false;
  float theta = (M_PI / 180) * -degree_theta;
  //fcl::Matrix3f rotation_mat = fcl::Matrix3f(1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta)); // X rotation
  //fcl::Matrix3f rotation_mat = fcl::Matrix3f(cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta)); // Y rotation
  fcl::Matrix3f rotation_mat = fcl::Matrix3f(cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1); // Z rotation

  fcl::Vec3f size = fcl::Vec3f(m_collision_distance, m_quad_width, m_quad_height);
  fcl::Box *box = new fcl::Box(size[0], size[1], size[2]);
  box->cost_density = 100;
  box->threshold_occupied = 5;

//  fcl::OcTree* tree = new fcl::OcTree(boost::shared_ptr<const octomap::OcTree>(octmap));
  fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(octmap));

  fcl::Vec3f box_center = fcl::Vec3f(m_collision_distance / 2 + m_min_distance, 0, 0);
  fcl::Vec3f translation_vec = fcl::Vec3f((box_center[0] * cos(theta) - box_center[1] * sin(theta)), (box_center[1] * cos(theta) + box_center[0] * sin(theta)), box_center[2]);
  fcl::Transform3f transformation = fcl::Transform3f(rotation_mat, translation_vec);
  collision_box = new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box), transformation);
  collision_tree = new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(tree), fcl::Transform3f());

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
  ros::Time start_avoid_time = ros::Time::now();
  int max_collision_angle = 90;
  int value_to_add = 5;
  bool willCollide = true;
  int counter = 0;


  ros::Time start_detect_time = ros::Time::now();
  willCollide = detect_collision(0);
  std_msgs::Float64 detect_time;
  detect_time.data = (ros::Time::now() - start_detect_time).toSec() * 1000;
  detect_time_pub.publish(detect_time);
  if (willCollide)
  {
      draw_new_direction(0, 0.5, true);
      for (int degree = 5; degree <= max_collision_angle; degree = degree + value_to_add)
      {
          counter++;
        if (detect_collision(degree))
        {
          //std::cout << "Collision detected" << std::endl;
//          draw_new_direction(-degree, 0.5, true);
        }
        else
        {
          //std::cout << "No collision" << std::endl;
          draw_new_direction(-degree - value_to_add, 4.0, false);
          willCollide = false;

          pub_safecopter_data(true, true, -degree - value_to_add);
          break;
        }

        if (detect_collision(-degree))
        {
          //std::cout << "Collision detected" << std::endl;
//          draw_new_direction(-degree, 0.5, true);
        }
        else
        {
          //std::cout << "No collision" << std::endl;
          draw_new_direction(degree + value_to_add, 4.0, false);
          willCollide = false;

          pub_safecopter_data(true, true, degree + value_to_add);
          break;
        }
      }
  }
  else
  {
      draw_new_direction(0, 4.0, false);
      pub_safecopter_data(false, true, 0);
  }
  
  //std::cout << "Counter: " << counter;
  if (willCollide)
  {
    draw_new_direction(0, 0.5, true);
    pub_safecopter_data(true, false, 0);
  }

  std_msgs::Float64 avoid_time;
  avoid_time.data = (ros::Time::now() - start_avoid_time).toSec() * 1000;
  avoid_time_pub.publish(avoid_time);
}

void colorize ()
{
  bool willCollide = false;
  int collidingPoints = 0;
  for (int i = 0; i < output_pcl.size(); i++)
  {
    pcl::PointXYZRGB point = output_pcl.at(i);
    double distance = sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));
    if (distance > 0 || distance < 0 || distance == 0) { // null test
      if (point.y < m_quad_width / 2 && point.y > -m_quad_width / 2 && distance < m_collision_distance && point.z < m_quad_height / 2 && point.z > -m_quad_height / 2 && distance > m_min_distance)
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
      else if (distance < m_min_distance)
      {

      }
      else if (distance < m_collision_distance)
      {
        point.r = 255;
        output_pcl.at(i) = point;
      }
      else if (distance < 2 * m_collision_distance)
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

void filter_ground ()
{
    ros::Time start_total_time = ros::Time::now();

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
        ROS_INFO_STREAM("Points before: " << output_pcl.size());
        pcl_ros::transformPointCloud (output_pcl, output_pcl, transform);

        pcl::PassThrough<pcl::PointXYZRGB> z_filter;
        z_filter.setFilterFieldName("z");
        z_filter.setFilterLimits(m_ground_minimum, 10);
        z_filter.setInputCloud(output_pcl.makeShared());
        z_filter.filter(output_pcl);
        ROS_INFO_STREAM("Points after: " << output_pcl.size());

//        pcl_ros::transformPointCloud (output_pcl, output_pcl, transform);
//        pcl::PointCloud<pcl::PointXYZRGB> temp_pc;
//        temp_pc.header.frame_id = base_link_id;
//        int j = 0;
//        for (int i = 0; i < output_pcl.size(); i++)
//        {
//            pcl::PointXYZRGB point = output_pcl.at(i);
//            if (point.z > 0 || point.z < 0 || point.z == 0) // null test
//             {
//                    if (point.z > m_ground_minimum)
//                    {
//                        j++;
//                        temp_pc.push_back(point);
//                    }
//             }
//        }
//        ROS_INFO_STREAM("Points: " << j);
//        output_pcl = temp_pc;

        try
        {
            tf_listener->lookupTransform(base_link_id, "odom", ros::Time(0), transform);
            pcl_ros::transformPointCloud (output_pcl, output_pcl, transform);
        }
        catch (tf::TransformException e)
        {
            ROS_ERROR_STREAM("Transform back to " << base_link_id << "failed " << e.what());
        }
    }

    std_msgs::Float64 total_time;
    total_time.data = (ros::Time::now() - start_total_time).toSec() * 1000;
    ground_time_pub.publish(total_time);
}

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

  if (m_filter_ground) {
      filter_ground();
  }

//  octomap::OcTree tree = octomap::OcTree(0.05);
//  octmap = &tree;

  ros::Time start_convert_time = ros::Time::now();
  octmap = new octomap::OcTree(0.05);
  octmap->setProbHit(0.9);
  octmap->setProbMiss(0.1);
  octmap->setClampingThresMin(0.49);
  octmap->setClampingThresMax(0.51);
  octomap::KeySet occupied_cells;

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
  std_msgs::Float64 convert_time;
  convert_time.data = (ros::Time::now() - start_convert_time).toSec() * 1000;
  convert_time_pub.publish(convert_time);

  colorize();

  avoid_collision();

  //detect_object();

  bool publishBinaryMap = (m_latchedTopics || binary_map_pub.getNumSubscribers() > 0);
  if (publishBinaryMap)
  {
      octomap_msgs::Octomap map;
      map.header.frame_id = base_link_id;
      map.header.stamp = ros::Time::now();
      if (octomap_msgs::binaryMapToMsg(*octmap, map))
      {
          binary_map_pub.publish(map);
      }
      else
      {
          ROS_ERROR("Error serializing OctoMap");
      }
  }

  pcl::toROSMsg(output_pcl, output);

  pub.publish(output);
  
  std_msgs::Float64 total_time;
  total_time.data = (ros::Time::now() - start_total_time).toSec() * 1000;
  total_time_pub.publish(total_time);

  delete octmap;
}

void cloud_cb_cam1 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!cam1_data_valid)
  {
//    std::cout << "Cam 1; " << endl;
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
//    std::cout << "Cam 2; " << endl;
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
//    std::cout << "Cam 3; " << endl;
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
  ros::init (argc, argv, "safecopter");
  ros::NodeHandle nh ("safecopter");

  // Define parameters
  nh.param("min_distance", m_min_distance, default_min_distance);
  nh.param("collision_distance", m_collision_distance, default_collision_distance);
  nh.param("quad_width", m_quad_width, default_quad_width);
  nh.param("quad_height", m_quad_height, default_quad_height);
  nh.param("ground_minimum", m_ground_minimum, default_ground_minimum);
  nh.param("filter_ground", m_filter_ground, default_filter_ground);
  print_param_changes();

  tf_listener = new tf::TransformListener();

  // Create a ROS subscriber for the input point cloud
  /*ros::Subscriber sub = nh.subscribe ("/pf1/points", 1, cloud_cb_cam1);
  ros::Subscriber sub2 = nh.subscribe ("/pf2/points", 1, cloud_cb_cam2);
  ros::Subscriber sub3 = nh.subscribe("/pf3/points", 1, cloud_cb_cam3);*/
  ros::Subscriber sub = nh.subscribe ("/pf_FF_dephcam/points", 1, cloud_cb_cam1);
  ros::Subscriber sub3 = nh.subscribe ("/pf_FR_dephcam/points", 1, cloud_cb_cam2);
  ros::Subscriber sub2 = nh.subscribe ("/pf_FL_dephcam/points", 1, cloud_cb_cam3);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_in", 1);
  new_direction_pub = nh.advertise<visualization_msgs::Marker>("new_direction", 1);
  vis_cube_pub = nh.advertise<visualization_msgs::Marker>("collision_box", 1);
  safecopter_pub = nh.advertise<std_msgs::Float32>("collision_free_angle", 1);
  will_collide_pub = nh.advertise<std_msgs::Bool>("will_collide", 1);
  can_find_path_pub = nh.advertise<std_msgs::Bool>("can_find_path", 1);
  combine_time_pub = nh.advertise<std_msgs::Float64>("/octree/combine_time", 1);
  detect_time_pub = nh.advertise<std_msgs::Float64>("/octree/detect_time", 1);
  avoid_time_pub = nh.advertise<std_msgs::Float64>("/octree/avoid_time", 1);
  convert_time_pub = nh.advertise<std_msgs::Float64>("/octree/convert_time", 1);
  ground_time_pub = nh.advertise<std_msgs::Float64>("/octree/ground_filter_time", 1);
  total_time_pub = nh.advertise<std_msgs::Float64>("/octree/total_time", 1);

  m_latchedTopics = true;
  if (m_latchedTopics)
  {
    ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
  } else
  {
    ROS_INFO("Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");
  }
  binary_map_pub = nh.advertise<octomap_msgs::Octomap>("octomap_binary", 5, m_latchedTopics);

  // Spin
  ros::spin ();
}
