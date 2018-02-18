#include "SceneAnalyzer.hpp"

// OpenCV
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"

double calculateSaliencyScore(const cv::KeyPoint &keypoint)
{
	const int x = keypoint.pt.x;
	const int y = keypoint.pt.y;
	const int delta = keypoint.octave;
	const double k = 0.01;

	return x * y * delta * k;
}

double calculateSaliencyMean(const DetectedPoints &points)
{
	double mean = 0;
	for (const auto& point : points)
		mean += calculateSaliencyScore(point);
	mean /= points.size();
	return mean;
}

double calculateSaliencySD(const DetectedPoints &points)
{
	double mean = calculateSaliencyMean(points);
	double sum_of_differences = 0;
	for (const auto& point : points)
		sum_of_differences += std::pow(calculateSaliencyScore(point) - mean, 2);
	return std::sqrt(((double)1 / (double)points.size()) * sum_of_differences);
}

DetectedPoints mostSalientKeypoints(DetectedPoints &keypoints)
{
	DetectedPoints salient_points;

	// Calculate the standard deviation of the keypoints, and erase those
	// that are lower than the standard deviation.
	double sd = calculateSaliencySD(salient_points);
	for (const auto &point : keypoints)
	{
		if (calculateSaliencyScore(point) >= sd)
		{
			salient_points.push_back(point);
		}
	}
	return salient_points;
}

SceneAnalyzer::SceneAnalyzer(const ros::ServiceClient &obj_detect_client,
                             float diag_fov)
{
	this->obj_detect_client = obj_detect_client;
	this->diag_fov = diag_fov;
}

SceneAnalyzer::ScenePoint SceneAnalyzer::next()
{
	auto point = points.front();
	points.pop();
	return point;
}

bool SceneAnalyzer::hasNext()
{
	return !points.empty();
}

void SceneAnalyzer::analyze(const SceneAnalyzer::SceneData &data)
{
	DetectedObjectsImgPair objs_img_pair = detectObjects(data);
	DetectedPoints salient_points = detectSalientPoints(data);

	// Convert the PCL ROS msg into a PointCloud before pulling points from it
	PointCloud cloud;
	pcl::fromROSMsg(*(data.cloud), cloud);

	// Attend objects first, then attend the salient points
	for (const auto &obj : objs_img_pair.first)
	{
		auto screen_pos = toScreen(obj);
		auto point = to3dPoint(screen_pos, data, cloud);
		points.push(point);
	}
	for (const auto &salient_point : salient_points)
	{
		auto screen_pos = toScreen(salient_point);
		auto point = to3dPoint(screen_pos, data, cloud);
		points.push(point);
	}
}

DetectedPoints SceneAnalyzer::detectSalientPoints(const SceneAnalyzer::SceneData &data)
{
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::ORB> detector = cv::ORB::create();
	detector->detect(cv_bridge::toCvCopy(data.image)->image, keypoints);
	return mostSalientKeypoints(keypoints);
}

DetectedObjectsImgPair SceneAnalyzer::detectObjects(const SceneAnalyzer::SceneData &data)
{
	tf_object_detection::ObjectDetection srv;
	srv.request.image = data.image;
	if (!obj_detect_client.call(srv))
	{
		ROS_WARN("Did not get a response from tf_object_detection service");
	}
	return {srv.response.detected_objects, srv.response.result_image};
}

SceneAnalyzer::ScenePoint SceneAnalyzer::to3dPoint(const SceneAnalyzer::ScreenPosition &screen,
                                                   const SceneAnalyzer::SceneData &data,
                                                   const PointCloud &cloud)
{
	// If the cloud is non-existent or disorganized, the PCL point can't be determined
	if (data.cloud == nullptr || !cloud.isOrganized())
	{
		return createFakePoint(screen,
		                       diag_fov, data.image.width, data.image.height);
	}

	// If the PCL point isn't finite, the 3D position can't be calculated
	pcl::PointXYZRGB point = cloud(screen.x, screen.y);
	if (!pcl::isFinite(point))
	{
		return createFakePoint(screen,
		                       diag_fov, data.image.width, data.image.height);
	}

	/*
	PCL point observations:

	The x-coordinate returned appears to be the depth, as when I took the center
	coordinates of the screen to be returned as a PCL point, this is the axis
	that changed significantly when depth was increased/decreased.

	As per the SO answer here:
	https://answers.ros.org/question/36142/points-in-a-pointcloud-and-their-distance-from-camera/
	It would appear that the y-axis is left and the z-axis is up.

	I really want to keep everything as a RHS-with-Y-up coordinate system to match
	the mesh analyser, so I'll swap the axes accordingly.
	*/
	float pcl_x = point.y;
	float pcl_y = point.z;
	float pcl_z = point.x;

	return {pcl_x, pcl_y, pcl_z};
}

SceneAnalyzer::ScreenPosition SceneAnalyzer::toScreen(const tf_object_detection::DetectedObject &object)
{
	// Determine the center point of the bounding box containing the object
	unsigned width = object.right - object.left;
	unsigned height = object.bottom - object.top;
	unsigned center_x = object.left + (width / 2);
	unsigned center_y = object.top + (height / 2);

	return {center_x, center_y};
}

SceneAnalyzer::ScreenPosition SceneAnalyzer::toScreen(const cv::KeyPoint &keypoint)
{
	return {keypoint.pt.x, keypoint.pt.y};
}

SceneAnalyzer::ScenePoint SceneAnalyzer::createFakePoint(const SceneAnalyzer::ScreenPosition &screen,
                                                         float diag_fov,
                                                         unsigned screen_width, unsigned screen_height)
{
	float diag_length = std::sqrt(std::pow(screen_width,2) + std::pow(screen_height,2));
	float rads_per_pixel = diag_fov / diag_length;

	unsigned screen_cx = screen_width / 2;
	unsigned screen_cy = screen_height / 2;

	float angle_x = (screen_cx - screen.x) * rads_per_pixel;
	float angle_y = (screen_cy - screen.y) * rads_per_pixel;

	SceneAnalyzer::ScenePoint fake_point;
	fake_point.z = 20.0f;
	fake_point.x = fake_point.z * tanf(angle_x);
	fake_point.y = fake_point.z * tanf(angle_y);

	return fake_point;
}