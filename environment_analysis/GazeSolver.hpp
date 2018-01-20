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

using DetectedKeypoints = std::vector<cv::KeyPoint>;
using DetectedObjects = std::vector<tf_object_detection::DetectedObject>;

class GazeSolver
{
public:
	GazeSolver(const ros::ServiceClient &obj_detect_client);

	void next(const sensor_msgs::Image &img_msg,
	          const sensor_msgs::PointCloud2Ptr &cloud,
	          const zed_wrapper::Mesh &mesh);

	GazeVisualizer &vis();

private:
	GaussianMap gaussian_map;
	MeshAnalyser mesh_analyser;
	GazeVisualizer visualizer;
	ros::ServiceClient obj_detect_client;

	DetectedKeypoints detectKeypoints();
	DetectedObjects detectObjects(const sensor_msgs::Image &img_msg);

	DetectedKeypoints mostSalientKeypoints(DetectedKeypoints &keypoints);
};

#endif // _GAZE_SOLVER_H_