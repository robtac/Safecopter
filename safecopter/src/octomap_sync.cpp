using namespace safecopter;

ros::Publisher pc_pub, odom_pub;
bool pc_valid = false;
bool odom_valid = false;

tf::TransformListener *tf_listener;

sensor_msgs::PointCloud2 cloud_in;

class OctomapSync
{
    void publish ()
    {
        pc_pub.publish(cloud_in);
    }

    void odom_cb (const )
    {

    }

    void pc_cb (const sensor_msgs::PointCloud2ConstPtr& input)
    {
      if (!pc_valid)
      {
        pc_valid = true;

        cloud_in = input;

        tf_listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
      }
    }
};

int main(int argc, char ** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "safecopter");
    ros::NodeHandle nh ("safecopter");

    tf_listener = new tf::TransformListener();

    ros::Subscriber pc_sub = nh.subscribe ("cloud_in", 1, pc_cb);
    ros::Subscriber odom_sub = nh.subscribe ("/odom", 1, odom_cb);

    // Create a ROS publisher for the output point cloud
    pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_in/sync", 1);
    odom_pub = nh.advertise<visualization_msgs::Marker>("/odom/sync", 1);

    // Spin
    ros::spin ();
}
