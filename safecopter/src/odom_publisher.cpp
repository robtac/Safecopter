#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

ros::Subscriber pos_sub;
//tf::TransformBroadcaster broadcaster;
geometry_msgs::TransformStamped odom_trans;

void pos_cb (const geometry_msgs::PoseStamped& input)
{
    static tf::TransformBroadcaster br;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = input.pose.position.x;
    odom_trans.transform.translation.y = input.pose.position.y;
    odom_trans.transform.translation.z = input.pose.position.z;
    odom_trans.transform.rotation = input.pose.orientation;
    br.sendTransform(odom_trans);
//    std::cout << "Printing" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle n;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    pos_sub = n.subscribe ("/mavros/local_position/pose", 1, pos_cb);

    ros::spin ();
}
