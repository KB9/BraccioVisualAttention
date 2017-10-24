// ROS
#include "ros/ros.h"

// Image
#include "sensor_msgs/Image.h"

// PCL
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

// Odometry
#include "nav_msgs/Odometry.h"

// OpenCV
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

/*
ROADMAP:

[x] Get the position and colors of all points in the cloud.
[x] Get the position and orientation of the camera.
[x] Get the live RGB image of the scene from the camera.
*/

void imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
	try
	{
		cv::imshow("view", cv_bridge::toCvShare(img_msg, "bgr8")->image);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
	}
}

void cloudMapCallback(const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
	// Declare the iterators for the point cloud data
	sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");

	// NOTE: I'm going to assume all iterator lengths are the same
	while (iter_x != iter_x.end())
	{
		ROS_INFO("POINTCLOUD2:");
		ROS_INFO("pos: (%.3f,%.3f,%.3f) rgb: (%u,%u,%u)",
			*iter_x, *iter_y, *iter_z, *iter_r, *iter_g, *iter_b);

		// Increment iterators to access the next point
		++iter_x;
		++iter_y;
		++iter_z;
		++iter_r;
		++iter_g;
		++iter_b;
	}
}

void positionCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	// Position as Euclidean coordinates
	double pos_x = odom_msg->pose.pose.position.x;
	double pos_y = odom_msg->pose.pose.position.y;
	double pos_z = odom_msg->pose.pose.position.z;

	// Orientation as quaternion
	double rot_x = odom_msg->pose.pose.orientation.x;
	double rot_y = odom_msg->pose.pose.orientation.y;
	double rot_z = odom_msg->pose.pose.orientation.z;
	double rot_w = odom_msg->pose.pose.orientation.w;

	ROS_INFO("ODOMETRY:");
	ROS_INFO("pos: (%.3f,%.3f,%.3f) orient: (%.3f,%.3f,%.3f,%.3f)",
		pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, rot_w);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rtabmap_ros_listener");
	ros::NodeHandle node_handle;
	ros::Rate loop_rate(10);

	ros::Subscriber img_sub;
	img_sub = node_handle.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback);

	ros::Subscriber pcl2_sub;
	pcl2_sub = node_handle.subscribe("/rtabmap/cloud_map", 1, cloudMapCallback);

	ros::Subscriber odom_sub;
	odom_sub = node_handle.subscribe("/rtabmap/odom", 1, positionCallback);

	ros::spin();

	return 0;
}