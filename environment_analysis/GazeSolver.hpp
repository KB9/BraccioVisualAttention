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

// FocusMapper
#include "FocusMapper.hpp"

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
	bool is_estimate;
};

class GazeSolver
{
public:
	GazeSolver(const ros::ServiceClient &obj_detect_client,
	           std::function<GazePoint(const GazePoint)>,
	           float diag_fov);

	GazePoint next(const SensorData &data);
	GazePoint findUnderMappedSection(const SensorData &data);

	void showVisualization(const sensor_msgs::Image &img_msg);

private:
	float diag_fov;
	ros::ServiceClient obj_detect_client;
	std::function<GazePoint(const GazePoint)> local_to_world;

	FocusMapper focus_mapper;
	MeshAnalyser mesh_analyser;
	GazeVisualizer visualizer;

	DetectedKeypoints detectKeypoints();
	DetectedObjects detectObjects(const sensor_msgs::Image &img_msg);
	DetectedKeypoints mostSalientKeypoints(std::vector<cv::KeyPoint> &keypoints);

	std::pair<tf_object_detection::DetectedObject, GazePoint> findBestObject(const DetectedObjects &objects,
	                                                                         const SensorData &data,
	                                                                         const PointCloud &cloud);
	std::pair<SalientPoint, GazePoint> findBestKeyPoint(const DetectedKeypoints &keypoints,
	                                                    const SensorData &data,
	                                                    const PointCloud &cloud);

	GazePoint find3dPoint(const ScreenPosition &screen, const SensorData &data,
	                      const PointCloud &cloud);
	ScreenPosition toScreen(const tf_object_detection::DetectedObject &object);
	ScreenPosition toScreen(const cv::KeyPoint &keypoint);

	GazePoint createFakePoint(const ScreenPosition &screen, float diag_fov,
	                          unsigned screen_width, unsigned screen_height);

	GazePoint rotate3dPoint(const GazePoint &point,
	                        float x_angle, float y_angle, float z_angle);

	float calculateFocusPenalty(const GazePoint &point);
	void addFocusPenalty(const GazePoint &point);
};

#endif // _GAZE_SOLVER_H_