#ifndef PC_COMBINE_H
#define PC_COMBINE_H

void detect_collision ();
void avoid_collision ();
void colorize ();
void pcl_combine ();
void cloud_cb_cam1 (const sensor_msgs::PointCloud2ConstPtr& input);
void cloud_cb_cam2 (const sensor_msgs::PointCloud2ConstPtr& input);
void cloud_cb_cam3 (const sensor_msgs::PointCloud2ConstPtr& input);

#endif
