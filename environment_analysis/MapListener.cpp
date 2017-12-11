// Infra
#include <vector>
#include <string>
#include <memory>
#include <cmath>

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

#include "GaussianMap.hpp"

Braccio braccio;
sensor_msgs::PointCloud2Ptr pcl_msg = nullptr;
std::vector<SalientPoint> salient_points;
GaussianMap gaussian_map;

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

	// Draw marker in center to indicate gaze focus point
	cv::circle(image, {image.cols/2,image.rows/2}, 1, {0,0,255,255});

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
	if (salient_points.size() == 0)
	{
		ROS_WARN("No salient points in view to focus on");
		return;
	}

	// The ZED camera has a diagonal FOV of 110 degrees
	constexpr double fov = toRadians(110.0f); // for the ZED camera
	constexpr double diag_length = std::sqrt((1280.0 * 1280.0) + (720.0 * 720.0));
	constexpr double radians_per_pixel = fov / diag_length;

	constexpr double center_x = 640.0;
	constexpr double center_y = 360.0;

	float best_point_dist = 0.0f;
	float best_x = 0.0f, best_y = 0.0f, best_z = 0.0f;
	for (auto &point : salient_points)
	{
		// Compute the horizontal and vertical angles required to reach the point
		double angle_horiz = (point.getCameraX() - center_x) * radians_per_pixel;
		double angle_vert = -(point.getCameraY() - center_y) * radians_per_pixel; // Be careful with the direction here...

		// Compute the 3D position of the effector after the rotation is applied
		const double eff_x = braccio.getEffectorX();
		const double eff_y = braccio.getEffectorY();
		const double eff_z = braccio.getEffectorZ();
		double new_eff_x = cosf(angle_vert)*eff_x + sinf(angle_vert)*sinf(angle_horiz)*eff_y - sinf(angle_vert)*cosf(angle_horiz)*eff_z;
		double new_eff_y = cosf(angle_horiz)*eff_y + sinf(angle_horiz)*eff_z;
		double new_eff_z = sinf(angle_vert)*eff_x + cosf(angle_vert)*-sinf(angle_horiz)*eff_y + cosf(angle_vert)*cosf(angle_horiz)*eff_z;

		// Extend these points to fake a sense of depth from the effector/origin
		const float depth_factor = 20.0f;
		new_eff_x *= depth_factor;
		new_eff_y *= depth_factor;
		new_eff_z *= depth_factor;

		// The best salient point to look at will be the point with the highest
		// saliency value and the lowest result from the gaussian computation.
		// This point can be found by adding the saliency score to the negated
		// gaussian result, and finding the absolute distance between these values
		// (This can also be achieved by multiplying the gaussian result by 2).
		float point_dist = point.getSaliencyScore() + (gaussian_map.calculate(new_eff_x, new_eff_y, new_eff_z) * 2.0f);
		if (point_dist > best_point_dist)
		{
			best_point_dist = point_dist;
			best_x = new_eff_x;
			best_y = new_eff_y;
			best_z = new_eff_z;
		}
	}

	// Focus the camera on the point with the highest score
	ROS_INFO("Best point to focus on: (%.2f,%.2f,%.2f)", best_x, best_y, best_z);
	ROS_INFO("Point had highest score: %.f", best_point_dist);
	bool ok = braccio.lookAt(best_x, best_y, best_z);
	if (ok)
	{
		// Decay all existing Gaussians, and add the new point to the map
		gaussian_map.decay(5.0f);
		gaussian_map.add(best_x, best_y, best_z, 1000.0f);
	}
}

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