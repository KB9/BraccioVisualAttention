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
SensorData gaze_data;

void onBraccioGazeFocusedCallback(std_msgs::Bool value)
{	
	// Find the next gaze point to focus on
	// NOTE: The gaze point solver works with a RHS-with-Y-up coordinate system
	GazePoint gaze_point = gaze_solver->next(gaze_data);

	// Swap the RHS-with-Y-up coordinate scheme into the Braccio kinematic's
	// LHS-with-Z-up coordinate scheme
	const float M_TO_CM = 100.0f;
	float braccio_x = gaze_point.z * M_TO_CM;
	float braccio_y = -gaze_point.x * M_TO_CM;
	float braccio_z = gaze_point.y * M_TO_CM;
	braccio.lookAt(braccio_x, braccio_y, braccio_z, ReferenceFrame::Effector);
}

void imageCallback(const sensor_msgs::Image &img_msg)
{
	gaze_data.image = img_msg;
	gaze_solver->showVisualization(img_msg);
}

void cloudMapCallback(const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
	gaze_data.cloud = cloud_msg;
}

// Display of spatial mesh vertices
void meshCallback(const zed_wrapper::Mesh& mesh_msg)
{
	gaze_data.mesh = mesh_msg;
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

	ros::Subscriber mesh_sub;
	mesh_sub = node_handle.subscribe("/zed/mesh", 1, meshCallback);

	// Set up this node as a client of the TensorFlow object_detection service
	ros::ServiceClient client = node_handle.serviceClient<tf_object_detection::ObjectDetection>("object_detection");

	// NEW
	gaze_solver = std::make_unique<GazeSolver>(client);

	braccio.initGazeFeedback(node_handle, onBraccioGazeFocusedCallback);
	braccio.lookAt(5.0f, 5.0f, 20.0f);

	ros::spin();

	return 0;
}