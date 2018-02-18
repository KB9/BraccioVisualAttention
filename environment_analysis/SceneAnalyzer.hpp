#ifndef _SCENE_ANALYZER_H_
#define _SCENE_ANALYZER_H_

#include <vector>
#include <queue>

// ROS
#include "ros/ros.h"

// Image
#include "sensor_msgs/Image.h"

// PCL
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"

// Mesh
#include "zed_wrapper/Mesh.h"

#include "SalientPoint.hpp"

// TensorFlow object_detection ROS service
#include "tf_object_detection/ObjectDetection.h"
#include "tf_object_detection/DetectedObject.h"

using DetectedPoints = std::vector<cv::KeyPoint>;
using DetectedObjects = std::vector<tf_object_detection::DetectedObject>;
using DetectedObjectsImgPair = std::pair<DetectedObjects, sensor_msgs::Image>;
using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

class SceneAnalyzer
{
public:
	struct SceneData
	{
		sensor_msgs::Image image;
		sensor_msgs::PointCloud2Ptr cloud;
		zed_wrapper::Mesh mesh;
	};

	struct ScenePoint
	{
		float x, y, z;
	};

	struct ScreenPosition
	{
		unsigned x, y;
	};

	SceneAnalyzer(const ros::ServiceClient &obj_detect_client,
	              float diag_fov);

	void analyze(const SceneAnalyzer::SceneData &data);

	ScenePoint next();
	bool hasNext();

private:
	ros::ServiceClient obj_detect_client;
	std::queue<ScenePoint> points;
	float diag_fov;

	DetectedPoints detectSalientPoints(const SceneAnalyzer::SceneData &data);
	DetectedObjectsImgPair detectObjects(const SceneAnalyzer::SceneData &data);

	SceneAnalyzer::ScenePoint to3dPoint(const SceneAnalyzer::ScreenPosition &screen,
	                                    const SceneAnalyzer::SceneData &data,
	                                    const PointCloud &cloud);
	SceneAnalyzer::ScreenPosition toScreen(const tf_object_detection::DetectedObject &object);
	SceneAnalyzer::ScreenPosition toScreen(const cv::KeyPoint &keypoint);

	SceneAnalyzer::ScenePoint createFakePoint(const SceneAnalyzer::ScreenPosition &screen,
	                                          float diag_fov,
	                                          unsigned screen_width, unsigned screen_height);
};

#endif // _SCENE_ANALYZER_H_