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
	// Get the y-rotation of the effector
	BraccioJointAngles angles = braccio.getJointAngles();
	float angle_eff_deg = angles.shoulder + (angles.elbow - 90) + (angles.wrist - 90);
	float angle_eff = toRadians(angle_eff_deg);

	ROS_INFO("angles are %f %f %f %f",angles.base, angles.shoulder, angles.elbow, angles.wrist);
	// Get the z-rotation of the effector
	float angle_base = toRadians(angles.base);
	
	// Find the next gaze point to focus on
	// NOTE: The gaze point solver works with a RHS-with-Y-up coordinate system
	GazePoint gaze_point = gaze_solver->next(gaze_data);

	// static int first = 0;

	// Rotate the gaze point after swapping the axes so that the coordinate system
	// matches that of the Braccio kinematics
	GazePoint braccio_gaze_point = gaze_solver->alignPoint({gaze_point.z, -gaze_point.y, gaze_point.x},
	                                                       0.0f, angle_eff, angle_base);

	//gaze_point = {0,0,0};

	// if (first == 0)
	// 	{
	// 		gaze_point = {0, 0, 1};
	// 		ROS_INFO("angles are %f %f %f %f",angles.base, angles.shoulder, angles.elbow, angles.wrist);
	// 		ROS_INFO("camera pos is %f %f %f", gaze_point.x, gaze_point.y, gaze_point.z);
	// 		ROS_INFO("braccio pos becomes %f %f %f", braccio_gaze_point.x, braccio_gaze_point.y, braccio_gaze_point.z);
	// 		ROS_INFO ("given effectors %f %f %f ", braccio.getEffectorX(), braccio.getEffectorY(), braccio.getEffectorZ());
	// 		return ;
	// 	}

	// first++;


	// Add the effector position to the gaze point
	const float M_TO_CM = 100.0f;
	float braccio_x = (braccio_gaze_point.x * M_TO_CM) + braccio.getEffectorX();
	float braccio_y = (braccio_gaze_point.y * M_TO_CM) + braccio.getEffectorY();
	float braccio_z = (braccio_gaze_point.z * M_TO_CM) + braccio.getEffectorZ();

	// Send the coordinates to the Braccio
	braccio.lookAt(braccio_x, braccio_y, braccio_z);
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