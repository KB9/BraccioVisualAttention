// Infra
#include <vector>
#include <string>

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
#include "opencv2/features2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "SalientPoint.hpp"
#include "kinematics/BraccioKinematics.hpp"

/*
ROADMAP:

[x] Get the position and colors of all points in the cloud.
[x] Get the position and orientation of the camera.
[x] Get the live RGB image of the scene from the camera.
[x] Get the SIFT features from the live RGB image.
[ ] Calculate the saliency score for each keypoint.
	- TODO: Should use three standard deviations.
	- TODO: x_offset, y_offset in saliency score should be from image center
[ ] Create a working memory pool for salient points.
	- How do I prevent salient points being re-added for the current gaze point,
	  and those same points once the gaze has been shifted?
	  - Lowe's matching algorithm (SIFT), most likely can be achieved
	    with ORB as well.
*/

double calculateSaliencyMean(const std::vector<SalientPoint>& points)
{
	double mean = 0;
	for (const auto& point : points)
		mean += point.getSaliencyScore();
	mean /= points.size();
	return mean;
}

double calculateSaliencySD(const std::vector<SalientPoint>& points)
{
	double mean = calculateSaliencyMean(points);
	double sum_of_differences = 0;
	for (const auto& point : points)
		sum_of_differences += std::pow(point.getSaliencyScore() - mean, 2);
	return std::sqrt(((double)1 / (double)points.size()) * sum_of_differences);
}

void imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
	// Convert from the ROS image message to an OpenCV image
	cv::Mat image = cv_bridge::toCvCopy(img_msg)->image;

	// Detect the ORB key points in the image
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::ORB> detector = cv::ORB::create();
	detector->detect(image, keypoints);

	// Create the vector containing the wrapped salient keypoints
	std::vector<SalientPoint> salient_points;
	std::for_each(keypoints.begin(), keypoints.end(),
		[&](cv::KeyPoint& k) { salient_points.emplace_back(k, 0, 0, 0); });

	// Calculate the standard deviation of the keypoints, and erase those
	// that are lower than the standard deviation.
	ROS_INFO("Total salient points: %lu", salient_points.size());
	double sd = calculateSaliencySD(salient_points);
	salient_points.erase(std::remove_if(salient_points.begin(), salient_points.end(),
		[&](SalientPoint& p) { return p.getSaliencyScore() < sd; }),
		salient_points.end());
	ROS_INFO("Thresholded salient points: %lu", salient_points.size());

	// Draw the surviving key points on the original image
	for (const auto& point : salient_points)
		cv::drawKeypoints(image, { point.getKeyPoint() }, image);

	// Display the image with key points
	cv::imshow("view", image);
	cv::waitKey(30);
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

void sendJointAngles(const std::vector<double> &angles)
{
	std::string command = "python /home/kavan/catkin_ws/src/BraccioVisualAttention/environment_analysis/gaze_control/braccio_gaze_controller.py ";
	for (const auto& angle : angles)
	{
		command += (std::to_string(angle) + " ");
	}
	system(command.c_str());
}

int main(int argc, char **argv)
{
	BraccioKinematics kinematics;
	srand(time(NULL));
	float x = rand() / (float)RAND_MAX * 50.0;
	float y = rand() / (float)RAND_MAX * 50.0;
	float z = rand() / (float)RAND_MAX * 50.0;

	ROS_INFO("x = %f, y = %f, z = %f", x,y,z);

	BraccioJointAngles angles;
	bool success = kinematics.lookAt(x, y, z, angles);
	if (success)
	{
		ROS_INFO("base = %f, shoulder = %f, elbow = %f, wrist = %f", angles.base, angles.shoulder, angles.elbow, angles.wrist);
		sendJointAngles({angles.base, angles.shoulder, angles.elbow, angles.wrist, angles.wrist_rot});
	}
	else
	{
		ROS_ERROR("Could not solve for (%.2f,%.2f,%.2f)", x, y, z);
	}

	ros::init(argc, argv, "rtabmap_ros_listener");
	ros::NodeHandle node_handle;
	ros::Rate loop_rate(10);

	ros::Subscriber img_sub;
	img_sub = node_handle.subscribe("/kinect2/qhd/image_color_rect", 1, imageCallback);

	//ros::Subscriber pcl2_sub;
	//pcl2_sub = node_handle.subscribe("/rtabmap/cloud_map", 1, cloudMapCallback);

	//ros::Subscriber odom_sub;
	//odom_sub = node_handle.subscribe("/rtabmap/odom", 1, positionCallback);

	ros::spin();

	return 0;
}