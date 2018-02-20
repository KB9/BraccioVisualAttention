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

#include "AsyncPeriodicRunner.hpp"

#include "zed_wrapper/SaveSpatialMap.h"

#include "GazeDirector.hpp"

Braccio braccio;

// NEW SYSTEMATIC ALGORITHM PLAN:
/*
The SphericalMapper will create a sphere of points to look out to. These can
be iterated over with its next() method. Once next() is called, the next() method
for the SceneAnalyzer should be called. Once SceneAnalyzer's next() stops
returning new points to look at, SphericalMapper's next() should be called. When
SphericalMapper's next() stops returning new points, the mesh should be saved.

This can be repeated ad infinitum if desired.
*/
std::unique_ptr<GazeDirector> gaze_director = nullptr;
SceneAnalyzer::SceneData scene_data;

SceneAnalyzer::ScenePoint gazeSolverToBraccio(const SceneAnalyzer::ScenePoint &gaze_solver_point)
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
	return {rx, ry, rz};
}

void onBraccioGazeFocusedCallback(std_msgs::Bool value)
{
	if (gaze_director->hasNext())
	{
		GazeDirector::GazePoint gaze_point = gaze_director->next(scene_data);
		braccio.lookAt(gaze_point.x, gaze_point.y, gaze_point.z);
	}
	else
	{
		// TODO: Save the mesh here. Repeat scanning if desired.
	}
}

void imageCallback(const sensor_msgs::Image &img_msg)
{
	scene_data.image = img_msg;
	gaze_director->visualize(scene_data);
}

void cloudMapCallback(const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
	scene_data.cloud = cloud_msg;
}

void meshCallback(const zed_wrapper::Mesh& mesh_msg)
{
	scene_data.mesh = mesh_msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper");
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

	gaze_director = std::make_unique<GazeDirector>(client, gazeSolverToBraccio, 1.9198621772f);

	braccio.initGazeFeedback(node_handle, onBraccioGazeFocusedCallback);
	braccio.lookAt(5.0f, 5.0f, 20.0f);

	ros::spin();

	return 0;
}