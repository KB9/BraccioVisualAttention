#ifndef _SCENE_ANALYZER_H_
#define _SCENE_ANALYZER_H_

#include <vector>
#include <deque>
#include <utility>
#include <string>

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

#include "FocusMapper.hpp"

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
		bool is_estimate;
		
		enum class Type
		{
			SALIENT,
			OBJECT
		};
		Type type;
		std::string description;
	};

	struct ScreenPosition
	{
		unsigned x, y;
	};

	SceneAnalyzer(const ros::ServiceClient &obj_detect_client,
	              std::function<ScenePoint(const ScenePoint &camera_point)> camera_to_world,
	              float diag_fov);

	void analyze(const SceneAnalyzer::SceneData &data);
	void visualize(const SceneAnalyzer::SceneData &data);

	ScenePoint next();
	bool hasNext();

private:
	ros::ServiceClient obj_detect_client;
	std::deque<ScenePoint> points;
	std::deque<ScenePoint> camera_points;

	std::deque<ScreenPosition> screen_points; // DEBUGGING

	float diag_fov;
	std::function<ScenePoint(const ScenePoint &camera_point)> camera_to_world;

	Eigen::MatrixXf perspective, analysis_pose;

	FocusMapper focus_mapper;

	DetectedPoints detectSalientPoints(const SceneAnalyzer::SceneData &data);
	DetectedObjectsImgPair detectObjects(const SceneAnalyzer::SceneData &data);

	void addScenePoint(const ScreenPosition &screen_pos, const SceneData &data,
	                   const PointCloud &cloud,
	                   const ScenePoint::Type &point_type,
	                   const std::string &description);

	SceneAnalyzer::ScenePoint to3dPoint(const SceneAnalyzer::ScreenPosition &screen,
	                                    const SceneAnalyzer::SceneData &data,
	                                    const PointCloud &cloud);
	SceneAnalyzer::ScreenPosition toScreen(const tf_object_detection::DetectedObject &object);
	SceneAnalyzer::ScreenPosition toScreen(const cv::KeyPoint &keypoint);

	SceneAnalyzer::ScenePoint createFakePoint(const SceneAnalyzer::ScreenPosition &screen,
	                                          float diag_fov,
	                                          unsigned screen_width, unsigned screen_height);

	void recordAnalysisPose(const SceneAnalyzer::SceneData &data);
	Eigen::MatrixXf getPoseTransformSinceAnalysis(const SceneAnalyzer::SceneData &data);
	Eigen::MatrixXf toEigenMatrix(const zed_wrapper::Matrix4f &msg);
};

#endif // _SCENE_ANALYZER_H_