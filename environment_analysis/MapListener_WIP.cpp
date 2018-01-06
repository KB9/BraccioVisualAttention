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

// Eigen
#include <Eigen/Core>

#include "SalientPoint.hpp"
#include "Braccio.hpp"

#include "GaussianMap.hpp"
#include "GazeVisualizer.hpp"

Braccio braccio;
sensor_msgs::PointCloud2Ptr pcl_msg = nullptr;
std::vector<SalientPoint> salient_points;
GaussianMap gaussian_map;

std::unique_ptr<GazeVisualizer> visualizer = nullptr;

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
	// Update the gaze visualizer with the latest image from the camera
	if (visualizer == nullptr)
		visualizer = std::make_unique<GazeVisualizer>(img_msg);
	else
		visualizer->update(img_msg);

	// Clear all existing salient point
	salient_points.clear();

	// Detect the key points in the image
	std::vector<cv::KeyPoint> keypoints = visualizer->detectKeyPoints();

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

	// Draw the keypoint that were found using the visualizer
	for (const auto& point : salient_points)
		visualizer->drawKeyPoint(point.getKeyPoint());

	// Display the result
	visualizer->show();
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

Pos3d toBraccioKinematicsAxes(Pos3d pos)
{
	return {pos.z, pos.x, -pos.y};
}

Pos3d calculateGazePointFromImage(float diag_fov, float width, float height,
                                  float x, float y)
{
	// I don't think this works. It's pretty wrong.

	// float diag_length = std::hypot(width, height);
	// float radians_per_pixel = diag_fov / diag_length;

	// float eff_x = braccio.getEffectorX();
	// float eff_y = braccio.getEffectorY();
	// float eff_z = braccio.getEffectorZ();

	// float center_x = width / 2.0f;
	// float center_y = height / 2.0f;

	// float angle_horiz = (x - center_x) * radians_per_pixel;
	// float angle_vert = (y - center_y) * radians_per_pixel;
	// ROS_INFO("Horiz angle: %.3f, vert angle: %.3f", angle_horiz, angle_vert);
	// ROS_INFO("Original effector pos: (%.3f,%.3f,%.3f)", eff_x, eff_y, eff_z);

	// Eigen::MatrixXf rot_y(3, 3);
	// rot_y(0, 0) = cosf(angle_vert);
	// rot_y(0, 1) = 0;
	// rot_y(0, 2) = sinf(angle_vert);
	// rot_y(1, 0) = 0;
	// rot_y(1, 1) = 1;
	// rot_y(1, 2) = 0;
	// rot_y(2, 0) = -sinf(angle_vert);
	// rot_y(2, 1) = 0;
	// rot_y(2, 2) = cosf(angle_vert);

	// Eigen::MatrixXf rot_z(3, 3);
	// rot_z(0, 0) = cosf(angle_horiz);
	// rot_z(0, 1) = -sinf(angle_horiz);
	// rot_z(0, 2) = 0;
	// rot_z(1, 0) = sinf(angle_horiz);
	// rot_z(1, 1) = cosf(angle_horiz);
	// rot_z(1, 2) = 0;
	// rot_z(2, 0) = 0;
	// rot_z(2, 1) = 0;
	// rot_z(2, 2) = 1;

	// Eigen::MatrixXf pos(3, 1);
	// pos(0, 0) = eff_x;
	// pos(1, 0) = eff_y;
	// pos(2, 0) = eff_z;

	// Eigen::MatrixXf result = rot_y * rot_z * pos;
	// float new_eff_x = result(0, 0);
	// float new_eff_y = result(1, 0);
	// float new_eff_z = result(2, 0);
	// ROS_INFO("New effector pos: (%.3f,%.3f,%.3f)", new_eff_x, new_eff_y, new_eff_z);

	// // Get the y-rotation of the effector
	// BraccioJointAngles angles = braccio.getJointAngles();
	// float angle_eff = angles.shoulder + (angles.elbow - 90.0f) + (angles.wrist - 90.0f);
	// angle_eff = toRadians(angle_eff);

	// // Work out the z-offset caused by the planar angle of the effector
	// float depth_factor = 10.0f;
	// new_eff_x *= depth_factor;
	// new_eff_y *= depth_factor;
	// new_eff_z += std::hypot(new_eff_x, new_eff_y) * tanf(angle_eff);

	// return {new_eff_x, new_eff_y, new_eff_z};

	float diag_length = std::hypot(width, height);
	float radians_per_pixel = diag_fov / diag_length;

	float eff_x = braccio.getEffectorX();
	float eff_y = braccio.getEffectorY();
	float eff_z = braccio.getEffectorZ();

	float center_x = width / 2.0f;
	float center_y = height / 2.0f;

	float angle_horiz = 0;//(x - center_x) * radians_per_pixel;
	float angle_vert = 0;//(y - center_y) * radians_per_pixel;

	// Create a point an extra 'depth_factor' units away from the base
	float depth_factor = 5.0f;

	// Get the y-rotation of the effector
	BraccioJointAngles angles = braccio.getJointAngles();
	float angle_eff_deg = angles.shoulder + (angles.elbow - 90.0f) + (angles.wrist - 90.0f);
	float angle_eff = toRadians(angle_eff_deg);

	// Get the z-rotation of the effector
	float angle_base = toRadians(angles.base);

	// =========================================================================
	// TODO:
	// Before rotating the point into base space, rotate the point based on the
	// horizontal and vertical angles present in the image.
	// =========================================================================
	// Since the point is to be extended outward from the effector, only increase the x-axis
	Eigen::MatrixXf pos(3, 1);
	pos(0, 0) = depth_factor;
	pos(1, 0) = 0.0f;
	pos(2, 0) = 0.0f;

	// Rotate the point according to the y-rotation and z-rotation of the salient
	// point in the 2D image
	Eigen::MatrixXf img_rot_y(3, 3);
	img_rot_y(0, 0) = cosf(angle_vert);
	img_rot_y(0, 1) = 0;
	img_rot_y(0, 2) = sinf(angle_vert);
	img_rot_y(1, 0) = 0;
	img_rot_y(1, 1) = 1;
	img_rot_y(1, 2) = 0;
	img_rot_y(2, 0) = -sinf(angle_vert);
	img_rot_y(2, 1) = 0;
	img_rot_y(2, 2) = cosf(angle_vert);

	Eigen::MatrixXf img_rot_z(3, 3);
	img_rot_z(0, 0) = cosf(angle_horiz);
	img_rot_z(0, 1) = -sinf(angle_horiz);
	img_rot_z(0, 2) = 0;
	img_rot_z(1, 0) = sinf(angle_horiz);
	img_rot_z(1, 1) = cosf(angle_horiz);
	img_rot_z(1, 2) = 0;
	img_rot_z(2, 0) = 0;
	img_rot_z(2, 1) = 0;
	img_rot_z(2, 2) = 1;

	Eigen::MatrixXf img_result = (img_rot_y * img_rot_z) * pos;
	// Eigen::MatrixXf img_result = pos;

	// =========================================================================

	// Rotate the point according to the y-rotation and z-rotation of the effector
	// using the effector as the origin
	Eigen::MatrixXf rot_y(3, 3);
	rot_y(0, 0) = cosf(angle_eff);
	rot_y(0, 1) = 0;
	rot_y(0, 2) = sinf(angle_eff);
	rot_y(1, 0) = 0;
	rot_y(1, 1) = 1;
	rot_y(1, 2) = 0;
	rot_y(2, 0) = -sinf(angle_eff);
	rot_y(2, 1) = 0;
	rot_y(2, 2) = cosf(angle_eff);

	Eigen::MatrixXf rot_z(3, 3);
	rot_z(0, 0) = cosf(angle_base);
	rot_z(0, 1) = -sinf(angle_base);
	rot_z(0, 2) = 0;
	rot_z(1, 0) = sinf(angle_base);
	rot_z(1, 1) = cosf(angle_base);
	rot_z(1, 2) = 0;
	rot_z(2, 0) = 0;
	rot_z(2, 1) = 0;
	rot_z(2, 2) = 1;

	// This is the point that the effector is focusing on in base space
	// Eigen::MatrixXf result = (rot_y * rot_z) * pos;
	Eigen::MatrixXf result = (rot_y * rot_z) * img_result;
	float focus_x = eff_x + result(0, 0);
	float focus_y = eff_y + result(1, 0);
	float focus_z = eff_z + result(2, 0);

	ROS_INFO("Salient angle H: %.3f, V: %.3f", angle_horiz, angle_vert);
	ROS_INFO("Effector pos: (%.3f,%.3f,%.3f)", eff_x, eff_y, eff_z);
	ROS_INFO("Effector angle: %.3f (%.3f degrees)", angle_eff, angle_eff_deg);
	ROS_INFO("Base angle: %.3f (%.3f degrees)", angle_base, angles.base);
	ROS_INFO("Looking at: (%.3f,%.3f,%.3f)", focus_x, focus_y, focus_z);
	ROS_INFO("Depth factor validation: %.3f", std::sqrt(std::pow(focus_x-eff_x,2) + std::pow(focus_y-eff_y,2) + std::pow(focus_z-eff_z,2)));
	ROS_INFO("========================================");

	return {focus_x, focus_y, focus_z};
}

Pos3d calculateGazePointFromPCL(pcl::PointXYZRGB point)
{
	// PCL points are in metres, convert them to centimetres for the kinematics
	float px = point.x * 100.0f;
	float py = point.y * 100.0f;
	float pz = point.z * 100.0f;

	auto pos = toBraccioKinematicsAxes({px, py, pz});
	auto br_pos = braccio.toBaseRelative(pos.x, pos.y, pos.z);

	return {br_pos.x, br_pos.y, br_pos.z};
}

// Pos3d test_point{25, 0, 25};
// bool x_axis_tested = false;
// bool y_axis_tested = false;
// bool z_axis_tested = false;

// void kinematicsTestCallback(std_msgs::Bool value)
// {
// 	bool ok = braccio.lookAt(test_point.x, test_point.y, test_point.z);
// 	if (!ok)
// 	{
// 		ROS_WARN("Kinematics test failed at (%.3f,%.3f,%.3f)", test_point.x, test_point.y, test_point.z);
// 	}
// 	else
// 	{
// 		ROS_INFO("Kinematics test at (%.3f,%.3f,%.3f)", test_point.x, test_point.y, test_point.z);
// 	}

// 	// Test the x-axis first
// 	if (!x_axis_tested)
// 	{
// 		if (test_point.x > -25)
// 		{
// 			test_point.x -= 5;
// 		}
// 		else
// 		{
// 			// Initialize the start point for y-axis testing
// 			test_point = {0, 25, 25};
// 			x_axis_tested = true;
// 		}
// 	}

// 	// Test the y-axis next
// 	if (x_axis_tested && !y_axis_tested)
// 	{
// 		if (test_point.y > -25)
// 		{
// 			test_point.y -= 5;
// 		}
// 		else
// 		{
// 			// Initialize the start point for y-axis testing
// 			test_point = {0, 0, 20};
// 			y_axis_tested = true;
// 		}
// 	}

// 	// Test the z-axis last
// 	if (x_axis_tested && y_axis_tested && !z_axis_tested)
// 	{
// 		if (test_point.z < 40)
// 		{
// 			test_point.z += 5;
// 		}
// 		else
// 		{
// 			z_axis_tested = true;
// 		}
// 	}
// }

void onBraccioGazeFocusedCallback(std_msgs::Bool value)
{
	if (salient_points.size() == 0)
	{
		ROS_WARN("No salient points in view to focus on");
		return;
	}

	// Convert the point cloud ROS message to a PointCloud object
	pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
	pcl::fromROSMsg(*pcl_msg, pcl_cloud);

	float best_point_dist = 0.0f;
	float best_x = 0.0f, best_y = 0.0f, best_z = 0.0f;
	bool using_pcl_point = false;
	for (auto &point : salient_points)
	{
		// If the PCL point is finite, use it to calculate the gaze point. If not,
		// fall back to calculating using the image and camera specs to estimate
		// the position.
		Pos3d point3d;
		// pcl::PointXYZRGB salient_pcl_point = getPCLPoint(pcl_cloud, point.getCameraX(), point.getCameraY());
		// if (pcl::isFinite(salient_pcl_point))
		// {
		// 	point3d = calculateGazePointFromPCL(salient_pcl_point);
		// 	using_pcl_point = true;
		// }
		// else
		// {
			float diag_fov = toRadians(75.0f);
			float width = 640.0f;
			float height = 480.0f;
			point3d = calculateGazePointFromImage(diag_fov, width, height,
			                                      point.getCameraX(),
			                                      point.getCameraY());
			using_pcl_point = false;
		// }

		// The best salient point to look at will be the point with the highest
		// saliency value and the lowest result from the gaussian computation.
		// This point can be found by adding the saliency score to the negated
		// gaussian result, and finding the absolute distance between these values
		// (This can also be achieved by multiplying the gaussian result by 2).
		float point_dist = point.getSaliencyScore() + (gaussian_map.calculate(point3d.x, point3d.y, point3d.z) * 2.0f);
		if (point_dist > best_point_dist)
		{
			best_point_dist = point_dist;
			best_x = point3d.x;
			best_y = point3d.y;
			best_z = point3d.z;

			visualizer->setGazePoint(point.getCameraX(), point.getCameraY());
		}
	}

	// Report the data that was used to calculate the new gaze point
	if (using_pcl_point)
		ROS_INFO("Using available PCL point to calculate gaze point.");
	else
		ROS_INFO("PCL point not available. Reverting to image to estimate gaze point.");

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
	img_sub = node_handle.subscribe("/camera/rgb/image_rect_color", 1, imageCallback);

	ros::Subscriber pcl2_sub;
	pcl2_sub = node_handle.subscribe("/camera/depth_registered/points", 1, cloudMapCallback);

	//ros::Subscriber odom_sub;
	//odom_sub = node_handle.subscribe("/zed/odom", 1, positionCallback);

	braccio.initGazeFeedback(node_handle, onBraccioGazeFocusedCallback);
	braccio.lookAt(5.0f, 0.0f, 20.0f);

	ros::spin();

	return 0;
}