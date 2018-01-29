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

GazePoint gazeSolverToBraccio(const GazePoint &gaze_solver_point)
{
	const float M_TO_CM = 100.0f;
	float braccio_x = gaze_solver_point.z * M_TO_CM;
	float braccio_y = -gaze_solver_point.x * M_TO_CM;
	float braccio_z = gaze_solver_point.y * M_TO_CM;

	// Get the angle of the base and effector in radians
	BraccioJointAngles angles = braccio.getJointAngles();
	float angle_eff_deg = angles.shoulder + (angles.elbow - 90) + (angles.wrist - 90);
	float angle_eff = toRadians(angle_eff_deg);
	float angle_base = toRadians(angles.base);

	// Initialize the two angles of rotation that are possible for the Braccio
	// effector
	float effector_y_angle = angle_eff;
	float effector_z_angle = -angle_base;

	Eigen::MatrixXf y_rotate(3,3);
	y_rotate(0,0) = cosf(effector_y_angle);
	y_rotate(0,1) = 0;
	y_rotate(0,2) = -sinf(effector_y_angle);
	y_rotate(1,0) = 0;
	y_rotate(1,1) = 1;
	y_rotate(1,2) = 0;
	y_rotate(2,0) = sinf(effector_y_angle);
	y_rotate(2,1) = 0;
	y_rotate(2,2) = cosf(effector_y_angle);

	Eigen::MatrixXf z_rotate(3,3);
	z_rotate(0,0) = cosf(effector_z_angle);
	z_rotate(0,1) = sinf(effector_z_angle);
	z_rotate(0,2) = 0;
	z_rotate(1,0) = -sinf(effector_z_angle);
	z_rotate(1,1) = cosf(effector_z_angle);
	z_rotate(1,2) = 0;
	z_rotate(2,0) = 0;
	z_rotate(2,1) = 0;
	z_rotate(2,2) = 1;

	Eigen::MatrixXf pos(3,1);
	pos(0,0) = braccio_x;
	pos(1,0) = braccio_y;
	pos(2,0) = braccio_z;

	// First rotate around Y then around Z, then add the effector positions to
	// make the base center the origin
	Eigen::MatrixXf result = z_rotate * (y_rotate * pos);
	float rx = result(0,0) + braccio.getEffectorX();
	float ry = result(1,0) + braccio.getEffectorY();
	float rz = result(2,0) + braccio.getEffectorZ();
	return {rx, ry, rz, gaze_solver_point.is_estimate};
}

void onBraccioGazeFocusedCallback(std_msgs::Bool value)
{	
	// Find the next gaze point to focus on
	// NOTE: The gaze point solver works with a RHS-with-Y-up coordinate system
	GazePoint gaze_point = gaze_solver->next(gaze_data);

	// Convert the RHS-with-Y-up scheme used by the gaze solver to the
	// LHS-with-Z-up coordinate scheme used by the Braccio kinematics solver
	GazePoint braccio_point = gazeSolverToBraccio(gaze_point);

	// Send the gaze point to the Braccio kinematics solver, and move the Braccio
	// to focus on it
	braccio.lookAt(braccio_point.x, braccio_point.y, braccio_point.z);
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
	gaze_solver = std::make_unique<GazeSolver>(client, 1.9198621772f);

	braccio.initGazeFeedback(node_handle, onBraccioGazeFocusedCallback);
	braccio.lookAt(5.0f, 5.0f, 20.0f);

	ros::spin();

	return 0;
}