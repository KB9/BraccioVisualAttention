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
#include "pcl_ros/point_cloud.h"

// Eigen
#include <Eigen/Core>

#include "Braccio.hpp"

// TensorFlow object_detection ROS service
#include "tf_object_detection/ObjectDetection.h"

#include "GazeSolver.hpp"

Braccio braccio;

std::unique_ptr<GazeSolver> gaze_solver = nullptr;
sensor_msgs::Image img;
sensor_msgs::PointCloud2Ptr pcl_msg;
zed_wrapper::Mesh mesh;

void onBraccioGazeFocusedCallback(std_msgs::Bool value)
{
	// Find the next gaze point to focus on
	// TODO: Make this return a result
	gaze_solver->next(img, pcl_msg, mesh);
}

void imageCallback(const sensor_msgs::Image &img_msg)
{
	img = img_msg;

	// TODO: Perform next gaze point solving every time an image is received, as
	// the Braccio is not currently being used
	onBraccioGazeFocusedCallback(std_msgs::Bool{});

	gaze_solver->vis().show();
}

void cloudMapCallback(const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
	pcl_msg = cloud_msg;
}

// Display of spatial mesh vertices
void meshCallback(const zed_wrapper::Mesh& mesh_msg)
{
	mesh = mesh_msg;
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

	// DISABLED: Braccio functionality disabled
	// braccio.initGazeFeedback(node_handle, onBraccioGazeFocusedCallback);
	// braccio.lookAt(5.0f, 5.0f, 20.0f);

	ros::Subscriber mesh_sub;
	mesh_sub = node_handle.subscribe("/zed/mesh", 1, meshCallback);

	// Set up this node as a client of the TensorFlow object_detection service
	ros::ServiceClient client = node_handle.serviceClient<tf_object_detection::ObjectDetection>("object_detection");

	// NEW
	gaze_solver = std::make_unique<GazeSolver>(client);

	ros::spin();

	return 0;
}