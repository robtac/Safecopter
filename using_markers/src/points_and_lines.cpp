#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "points_and_lines.h"

#include <cmath>

void drawCircle (float radius, int rColour, int gColour, int bColour, ros::Publisher marker_pub)
{
    visualization_msgs::Marker points, line_strip;
  
    points.header.frame_id = line_strip.header.frame_id = "base_link";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "points_and_lines";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w;
    
    points.id = 0;
    line_strip.id = 1;
    
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.01;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.r = rColour;
    line_strip.color.g = gColour;
    line_strip.color.b = bColour;
    line_strip.color.a = 1.0;

    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 73; ++i)
    {
      float x = radius * sin(i / 72.0f * 2 * M_PI);
      float y = radius * cos(i / 72.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = x;
      p.y = y;
      p.z = (float) 0;

      points.points.push_back(p);
      line_strip.points.push_back(p);
      
      marker_pub.publish(line_strip);
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("circle_marker_inside", 1);
  ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("circle_marker_outside", 1);
  ros::Rate r(30);
  while (ros::ok())
  {
    drawCircle(1.0, 1, 0, 0, marker_pub);
    drawCircle(2.0, 0, 1, 0, marker_pub2);
    
    //marker_pub.publish(points);

    r.sleep();
  }
}