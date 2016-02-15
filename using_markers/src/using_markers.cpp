#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker targetArrow ()
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

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
  marker.pose.orientation.z = 23.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 10.0;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  
  return marker;
}

visualization_msgs::Marker actualArrow ()
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

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
  marker.pose.orientation.z = 27.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 10.0;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  
  return marker;
}

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
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub_target = n.advertise<visualization_msgs::Marker>("targetArrow", 1);
  ros::Publisher marker_pub_actual = n.advertise<visualization_msgs::Marker>("actualArrow", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("circle_inside", 1);
  ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("circle_outside", 1);

  while (ros::ok())
  {
    
    visualization_msgs::Marker targetMarker = targetArrow();
    visualization_msgs::Marker actualMarker = actualArrow();
    targetMarker.lifetime = ros::Duration();
    actualMarker.lifetime = ros::Duration();
    
    drawCircle(1.0, 1, 0, 0, marker_pub);
    drawCircle(2.0, 0, 1, 0, marker_pub2);
    
    marker_pub_target.publish(targetMarker);
    marker_pub_actual.publish(actualMarker);

    r.sleep();
  }
}