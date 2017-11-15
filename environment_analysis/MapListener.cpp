// Infra
#include <vector>

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

// kdl_parser
#include <kdl_parser/kdl_parser.hpp>

// Orocos KDL
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

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

bool calculateJointAngles(double x, double y, double z, std::vector<double> &angles)
{
	// Load the KDL tree from the Braccio URDF file
	KDL::Tree tree;
	if (!kdl_parser::treeFromFile("/home/kavan/catkin_ws/src/BraccioVisualAttention/environment_analysis/braccio.urdf", tree))
	{
		ROS_ERROR("Failed to construct KDL tree");
		return false;
	}

	// Print some stats about the KDL tree
	ROS_INFO("KDL::Tree joint count: %u", tree.getNrOfJoints());
	for (const auto& segment : tree.getSegments())
	{
		ROS_INFO("KDL::Segment name: %s", segment.first.c_str());
	}

	// Get the chain from the tree
	KDL::Chain chain;
	if (!tree.getChain("base_link", "braccio_ee", chain))
	{
		ROS_ERROR("Failed to create KDL chain");
		return false;
	}

	// Creation of the solvers
	KDL::ChainFkSolverPos_recursive fksolver(chain); // Forward position solver
	KDL::ChainIkSolverVel_pinv iksolverv(chain); // Inverse velocity solver
	const unsigned int max_iterations = 100;
	const double accuracy = 1e-6;
	KDL::ChainIkSolverPos_NR iksolver(chain, fksolver, iksolverv, 100, accuracy);

	// Creation of joint arrays
	KDL::JntArray q(chain.getNrOfJoints());
	KDL::JntArray q_init(chain.getNrOfJoints());

	// Set destination frame
	KDL::Frame f_dest(KDL::Vector(x, y, z));

	// Convert from cartesian position to joint angles
	int result = iksolver.CartToJnt(q_init, f_dest, q);
	ROS_INFO("JntArray rows: %u", q.rows());
	ROS_INFO("JntArray cols: %u", q.columns());
	ROS_INFO("Angles: %f, %f, %f, %f, %f", q.data[0], q.data[1], q.data[2], q.data[3], q.data[4]);
	for (unsigned int i = 0; i < q.rows(); i++)
		angles.push_back(q.data[i]);

	if (result == iksolver.E_NOERROR)
	{
		ROS_INFO("Joint positions calculated successfully!");
		return true;
	}
	else if (result == iksolver.E_DEGRADED)
	{
		ROS_INFO("Joint positions calculated - solution quality degraded");
		return true;
	}
	else if (result == iksolver.E_IKSOLVER_FAILED)
	{
		ROS_ERROR("Velocity solver failed");
		return false;
	}
	else if (result == iksolver.E_NO_CONVERGE)
	{
		ROS_ERROR("Solution did not converge");
		return false;
	}

	return false;
}

int main(int argc, char **argv)
{
	std::vector<double> angles;
	calculateJointAngles(5.0, 5.0, 5.0, angles);

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