// Infra
#include <vector>
#include <string>
#include <memory>

// ROS
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Bool.h"

// Image
#include "sensor_msgs/Image.h"

// PCL
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "pcl_ros/point_cloud.h"

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
#include "Braccio.hpp"

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

Braccio braccio;
sensor_msgs::PointCloud2Ptr pcl_msg = nullptr;
std::vector<SalientPoint> salient_points;

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
	salient_points.clear();

	// Convert from the ROS image message to an OpenCV image
	cv::Mat image = cv_bridge::toCvCopy(img_msg)->image;

	// Detect the ORB key points in the image
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::ORB> detector = cv::ORB::create();
	detector->detect(image, keypoints);

	// Create the vector containing the wrapped salient keypoints
	// std::vector<SalientPoint> salient_points;
	std::for_each(keypoints.begin(), keypoints.end(), [&](cv::KeyPoint& k)
	{
		salient_points.emplace_back(k, k.pt.x, k.pt.y, 0);
	});

	// Calculate the standard deviation of the keypoints, and erase those
	// that are lower than the standard deviation.
	double sd = calculateSaliencySD(salient_points);
	salient_points.erase(std::remove_if(salient_points.begin(), salient_points.end(),
		[&](SalientPoint& p) { return p.getSaliencyScore() < sd; }),
		salient_points.end());

	// TESTING: REMOVE ALL SALIENT POINTS APART FROM ONE!
	salient_points.erase(++salient_points.begin(), salient_points.end());

	// Draw the surviving key points on the original image
	for (const auto& point : salient_points)
		cv::drawKeypoints(image, { point.getKeyPoint() }, image);

	// Display the image with key points
	cv::imshow("view", image);
	cv::waitKey(30);
}

void cloudMapCallback(const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
	pcl_msg = cloud_msg;
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

pcl::PointXYZRGB getPCLPoint(pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud,
                             float x, float y)
{
	if (pcl_cloud.isOrganized())
	{
		return pcl_cloud(x, y);
	}
	else
	{
		ROS_WARN("Point cloud is not organized - implementation not defined.");
	}
}

Pos3d fromZedCameraAxis(Pos3d pos)
{
	return {pos.z, pos.x, -pos.y};
}

void onBraccioGazeFocusedCallback(std_msgs::Bool value)
{
	// Perform a deep copy to prevent this being modified whilst this callback
	// is running (TODO: Check if this is required as callbacks may be executed
	// concurrently).
	auto salient_points_copy = salient_points;

	ROS_INFO("Salient points: %lu", salient_points_copy.size());

	// TODO: There should only be one salient point for now to test that this
	// works. Later, it should be able to handle any number of points.
	if (salient_points_copy.size() != 1)
	{
		ROS_WARN("For testing, focusing on anything other than one salient point has been disabled!");
		return;
	}
	for (auto &point : salient_points_copy)
	{
		// Get the x,y coordinate to search for the point cloud
		double x = point.getCameraX();
		double y = point.getCameraY();
		ROS_INFO("Salient point: (%.3f,%.3f)", x, y);

		// Convert the point cloud ROS message to a PointCloud object
		pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
		pcl::fromROSMsg(*pcl_msg, pcl_cloud);

		// Get the salient point in 3D from the point cloud
		pcl::PointXYZRGB salient_pcl_point = getPCLPoint(pcl_cloud, x, y);
		ROS_INFO("PointCloud salient point: (%.3f,%.3f,%.3f)", salient_pcl_point.x, salient_pcl_point.y, salient_pcl_point.z);

		// NOTE: PCL points are in metres, convert them to centimetres for the kinematics

		ROS_INFO("TESTING ORIGIN:");
		auto pos = fromZedCameraAxis({0.0f, 0.0f, 0.0f});
		braccio.lookAt_BR(pos.x, pos.y, pos.z);

		ROS_INFO("TESTING Z:");
		pos = fromZedCameraAxis({0.0f, 0.0f, 1.0f});
		braccio.lookAt_BR(pos.x, pos.y, pos.z);
		pos = fromZedCameraAxis({0.0f, 0.0f, 2.0f});
		braccio.lookAt_BR(pos.x, pos.y, pos.z);
		pos = fromZedCameraAxis({0.0f, 0.0f, 3.0f});
		braccio.lookAt_BR(pos.x, pos.y, pos.z);

		ROS_INFO("TESTING Y:");
		pos = fromZedCameraAxis({0.0f, 1.0f, 0.0f});
		braccio.lookAt_BR(pos.x, pos.y, pos.z);
		pos = fromZedCameraAxis({0.0f, 2.0f, 0.0f});
		braccio.lookAt_BR(pos.x, pos.y, pos.z);
		pos = fromZedCameraAxis({0.0f, 3.0f, 0.0f});
		braccio.lookAt_BR(pos.x, pos.y, pos.z);

		ROS_INFO("TESTING X:");
		pos = fromZedCameraAxis({1.0f, 0.0f, 0.0f});
		braccio.lookAt_BR(pos.x, pos.y, pos.z);
		pos = fromZedCameraAxis({2.0f, 0.0f, 0.0f});
		braccio.lookAt_BR(pos.x, pos.y, pos.z);
		pos = fromZedCameraAxis({3.0f, 0.0f, 0.0f});
		braccio.lookAt_BR(pos.x, pos.y, pos.z);
	}
}

/*
NOTES:

The PointCloud<T> data structure appears to have a public 4-element vector
of floats called sensor_origin_.
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rtabmap_ros_listener");
	ros::NodeHandle node_handle;
	ros::Rate loop_rate(10);

	// REMINDER: Execute "roslaunch zed_wrapper zed.launch" to start receiving input

	ros::Subscriber img_sub;
	img_sub = node_handle.subscribe("/zed/rgb/image_rect_color", 1, imageCallback);

	ros::Subscriber pcl2_sub;
	pcl2_sub = node_handle.subscribe("/zed/point_cloud/cloud_registered", 1, cloudMapCallback);

	//ros::Subscriber odom_sub;
	//odom_sub = node_handle.subscribe("/zed/odom", 1, positionCallback);

	braccio.initGazeFeedback(node_handle, onBraccioGazeFocusedCallback);
	braccio.lookAt(5.0f, 0.0f, 20.0f);

	ros::spin();

	return 0;
}