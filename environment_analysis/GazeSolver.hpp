#ifndef _GAZE_SOLVER_H_
#define _GAZE_SOLVER_H_

// Infra
#include <vector>

// ROS
#include "ros/ros.h"

// Image
#include "sensor_msgs/Image.h"

// PCL
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"

// Mesh
#include "zed_wrapper/Mesh.h"

// TensorFlow object_detection ROS service
#include "tf_object_detection/ObjectDetection.h"
#include "tf_object_detection/DetectedObject.h"

// SalientPoint
#include "SalientPoint.hpp"

// GaussianMap
#include "GaussianMap.hpp"

// GazeVisualizer
#include "GazeVisualizer.hpp"

// MeshProjector
#include "MeshAnalyser.hpp"

using DetectedKeypoints = std::vector<SalientPoint>;
using DetectedObjects = std::vector<tf_object_detection::DetectedObject>;

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

struct SensorData
{
	sensor_msgs::Image image;
	sensor_msgs::PointCloud2Ptr cloud;
	zed_wrapper::Mesh mesh;
};

struct ScreenPosition
{
	unsigned x, y;
};

struct GazePoint
{
	float x, y, z;
};

class GazeSolver
{
public:
	GazeSolver(const ros::ServiceClient &obj_detect_client);

	GazePoint next(const SensorData &data);

	void showVisualization(const sensor_msgs::Image &img_msg);

private:
	GaussianMap gaussian_map;
	MeshAnalyser mesh_analyser;
	GazeVisualizer visualizer;
	ros::ServiceClient obj_detect_client;

	DetectedKeypoints detectKeypoints();
	DetectedObjects detectObjects(const sensor_msgs::Image &img_msg);
	DetectedKeypoints mostSalientKeypoints(std::vector<cv::KeyPoint> &keypoints);

	GazePoint find3dPoint(const ScreenPosition &screen, const SensorData &data);
	ScreenPosition toScreen(const tf_object_detection::DetectedObject &object);
	ScreenPosition toScreen(const cv::KeyPoint &keypoint);

	GazePoint rotate3dPoint(const GazePoint &point,
	                        float x_angle, float y_angle, float z_angle);
};

#endif // _GAZE_SOLVER_H_