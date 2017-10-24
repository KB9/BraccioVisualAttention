#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void cloudMapCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	ROS_INFO("PointCloud2 received");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_listener");
	ros::NodeHandle node_handle;
	ros::Rate loop_rate(10);
	ros::Subscriber sub;
	sub = node_handle.subscribe("/rtabmap/cloud_map", 1, cloudMapCallback);
	ros::spin();

	return 0;
}