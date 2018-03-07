#include "SceneAnalyzer.hpp"

#include <string>

// OpenCV
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"

// Eigen
#include <Eigen/Core>

#include <opencv2/saliency.hpp>

SceneAnalyzer::SceneAnalyzer(const ros::ServiceClient &obj_detect_client,
                             std::function<SceneAnalyzer::ScenePoint(const SceneAnalyzer::ScenePoint &camera_point)> camera_to_world,
                             float diag_fov)
{
	this->obj_detect_client = obj_detect_client;
	this->diag_fov = diag_fov;
	this->camera_to_world = camera_to_world;
}

SceneAnalyzer::ScenePoint SceneAnalyzer::next()
{
	auto point = points.front();
	points.pop_front();
	camera_points.pop_front();

	return point;
}

bool SceneAnalyzer::hasNext()
{
	return !points.empty();
}

void SceneAnalyzer::analyze(const SceneAnalyzer::SceneData &data)
{
	// Clear the last queue of points that were detected
	points.clear();
	camera_points.clear();
	focus_mapper.clear();

	DetectedObjectsImgPair objs_img_pair = detectObjects(data);
	DetectedPoints salient_points = detectSalientPoints(data);

	// Convert the PCL ROS msg into a PointCloud before pulling points from it
	PointCloud cloud;
	pcl::fromROSMsg(*(data.cloud), cloud);

	// Attend objects first, then attend the salient points
	for (const auto &obj : objs_img_pair.first)
	{
		auto screen_pos = toScreen(obj);
		addScenePoint(screen_pos, data, cloud, ScenePoint::Type::OBJECT, "Object: " + obj.obj_class);
		ROS_INFO("Object detected: %s", obj.obj_class.c_str());
	}
	for (const auto &salient_point : salient_points)
	{
		auto screen_pos = toScreen(salient_point);
		addScenePoint(screen_pos, data, cloud, ScenePoint::Type::SALIENT, "Salient point");
	}

	recordAnalysisPose(data);
}

void SceneAnalyzer::addScenePoint(const ScreenPosition &screen_pos,
                                  const SceneData &data,
                                  const PointCloud &cloud,
                                  const ScenePoint::Type &point_type,
                                  const std::string &description)
{
	const float FOCUS_THRESHOLD = 0.5f;
	const float FOCUS_RADIUS = 100.0f;

	auto point = to3dPoint(screen_pos, data, cloud);
	auto world_point = camera_to_world(point);
	float score = focus_mapper.calculate(world_point.x, world_point.y, world_point.z);
	if (score < FOCUS_THRESHOLD)
	{
		point.type = point_type;
		point.description = description;
		world_point.type = point_type;
		world_point.description = description;

		camera_points.push_back(point);
		points.push_back(world_point);
		focus_mapper.add(world_point.x, world_point.y, world_point.z, FOCUS_RADIUS);
	}
}

DetectedPoints SceneAnalyzer::detectSalientPoints(const SceneAnalyzer::SceneData &data)
{
	DetectedPoints detected_points;

	cv::Mat image = cv_bridge::toCvCopy(data.image)->image;
	cv::Mat saliency_map;
	cv::Mat binary_map;

	// Blur the image before computing the saliency
	cv::GaussianBlur(image, image, {0, 0}, 10);

	cv::Ptr<cv::saliency::Saliency> saliency_algorithm = cv::saliency::Saliency::create("SPECTRAL_RESIDUAL");
	if (saliency_algorithm->computeSaliency(image, saliency_map))
	{
		cv::saliency::StaticSaliencySpectralResidual spec;
		spec.computeBinaryMap(saliency_map, binary_map);

		cv::SimpleBlobDetector::Params params;
		params.blobColor = 255;
		params.filterByColor = true;
		params.minCircularity = 0.0;
		params.maxCircularity = 1.0;
		params.filterByCircularity = true;
		params.minConvexity = 0.0;
		params.maxConvexity = 1.0;
		params.filterByConvexity = true;
		params.filterByArea = false;
		params.filterByInertia = false;

		cv::Ptr<cv::SimpleBlobDetector> blob_detector = cv::SimpleBlobDetector::create(params);
		blob_detector->detect(binary_map, detected_points);
	}

	return detected_points;
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
	bool is_estimate = false;

	return {pcl_x, pcl_y, pcl_z, is_estimate};
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

	fake_point.is_estimate = true;

	return fake_point;
}

void SceneAnalyzer::visualize(const SceneAnalyzer::SceneData &data)
{
	cv::Mat cv_image = cv_bridge::toCvCopy(data.image)->image;

	if (!camera_points.empty())
	{
		Eigen::MatrixXf pose = getPoseTransformSinceAnalysis(data);
		Eigen::MatrixXf projection = perspective * pose.inverse();
		bool is_goal_point_color_set = false;
		float width = cv_image.cols;
		float height = cv_image.rows;

		for (const auto &point : camera_points)
		{
			// Camera points are stored in a RHS-with-Y-up coordinate system.
			// However, points on a screen are displayed in a RHS-with-Y-down
			// coordinate system therefore x and y coordinates need to be
			// flipped.
			Eigen::MatrixXf v(4,1);
			v(0,0) = -point.x;
			v(1,0) = -point.y;
			v(2,0) = point.z;
			v(3,0) = 1.0f;

			Eigen::MatrixXf result = projection * v;
			float x = result(0,0);
			float y = result(1,0);
			float z = result(2,0);
			float w = result(3,0);

			if (w >= 0.0f)
			{
				// Convert the 3D position to the 2D screen position
				float half_width = width / 2.0f;
				float half_height = height / 2.0f;
				float screen_x = (x * width) / (2.0f * w) + half_width;
				float screen_y = (y * height) / (2.0f * w) + half_height;

				if (!is_goal_point_color_set)
				{
					cv::circle(cv_image, {screen_x, screen_y}, 10, {0,0,255,255}, 4);
					cv::putText(cv_image, point.description, {screen_x, screen_y - 5}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,0,255,255});
				}
				else
				{
					cv::circle(cv_image, {screen_x, screen_y}, 5, {0,255,0,255}, 2);
					cv::putText(cv_image, point.description, {screen_x, screen_y - 5}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0,255});
				}
			}
			is_goal_point_color_set = true;
		}
	}

	// Draw red line overlay to indicate center of image
	cv::line(cv_image, {cv_image.cols/2,0}, {cv_image.cols/2,cv_image.rows}, {0,0,255,255});
	cv::line(cv_image, {0,cv_image.rows/2}, {cv_image.cols,cv_image.rows/2}, {0,0,255,255});

	cv::putText(cv_image, "Points to attend: " + std::to_string(points.size()),
	            {5, cv_image.rows-5}, cv::FONT_HERSHEY_SIMPLEX, 1, {255,255,255,255});

	// Display the image
	cv::imshow("SceneAnalyzer", cv_image);
	cv::waitKey(30);
}

Eigen::MatrixXf SceneAnalyzer::toEigenMatrix(const zed_wrapper::Matrix4f &msg)
{
	Eigen::MatrixXf matrix(4,4);
	matrix(0,0) = msg.m11;
	matrix(0,1) = msg.m12;
	matrix(0,2) = msg.m13;
	matrix(0,3) = msg.m14;
	matrix(1,0) = msg.m21;
	matrix(1,1) = msg.m22;
	matrix(1,2) = msg.m23;
	matrix(1,3) = msg.m24;
	matrix(2,0) = msg.m31;
	matrix(2,1) = msg.m32;
	matrix(2,2) = msg.m33;
	matrix(2,3) = msg.m34;
	matrix(3,0) = msg.m41;
	matrix(3,1) = msg.m42;
	matrix(3,2) = msg.m43;
	matrix(3,3) = msg.m44;
	return matrix;
}

void SceneAnalyzer::recordAnalysisPose(const SceneAnalyzer::SceneData &data)
{
	perspective = toEigenMatrix(data.mesh.perspective);
	analysis_pose = toEigenMatrix(data.mesh.pose);
}

Eigen::MatrixXf SceneAnalyzer::getPoseTransformSinceAnalysis(const SceneAnalyzer::SceneData &data)
{
	return analysis_pose.inverse() * toEigenMatrix(data.mesh.pose);
}