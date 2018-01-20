#include "GazeSolver.hpp"

// OpenCV
#include <opencv2/opencv.hpp>

// Eigen
#include <Eigen/Core>

double calculateSaliencyMean(const std::vector<SalientPoint>& points)
{
	double mean = 0;
	for (const auto& point : points)
		mean += point.getSaliencyScore();
	mean /= points.size();
	return mean;
}

double calculateSaliencySD(const std::vector<SalientPoint>& points)
{
	double mean = calculateSaliencyMean(points);
	double sum_of_differences = 0;
	for (const auto& point : points)
		sum_of_differences += std::pow(point.getSaliencyScore() - mean, 2);
	return std::sqrt(((double)1 / (double)points.size()) * sum_of_differences);
}

Eigen::MatrixXf toEigenMatrix(const zed_wrapper::Matrix4f &msg)
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

GazeSolver::GazeSolver(const ros::ServiceClient &obj_detect_client)
{
	this->obj_detect_client = obj_detect_client;
}

void GazeSolver::next(const sensor_msgs::Image &img_msg,
                      const sensor_msgs::PointCloud2Ptr &cloud,
                      const zed_wrapper::Mesh &mesh)
{
	// Update the gaze visualizer with the latest image from the camera
	visualizer.update(img_msg);

	// Detect the keypoints and objects in the image (keypoint detection must
	// occur before objects, as the object detection will update the vis image)
	DetectedKeypoints keypoints = detectKeypoints();
	DetectedObjects objects = detectObjects(img_msg);

	// Draw the keypoints on the image
	for (const auto &kp : keypoints)
		visualizer.drawKeyPoint(kp);

	// Set the perspective and pose matrices used to project the mesh vertices
	const Eigen::MatrixXf perspective = toEigenMatrix(mesh.perspective);
	const Eigen::MatrixXf pose = toEigenMatrix(mesh.pose);
	mesh_analyser.setPerspective(perspective);
	mesh_analyser.setPose(pose);

	// Register all mesh vertices with the mesh projector
	mesh_analyser.clearMesh();
	for (auto &chunk : mesh.chunks)
	{
		for (auto &v : chunk.vertices)
		{
			mesh_analyser.addVertex(v.x, v.y, v.z);
		}
	}

	// Find the angles required to view an undermapped section of the mesh
	Rotation rot = mesh_analyser.findLesserMappedSection();
	ROS_INFO("Required: (%.3f,%.3f,%.3f)", rot.x, rot.y, rot.z);

	/*
	PLAN FOR SOLVER:
	When the mapping starts, try and locate salient points and objects in the
	current FOV. This will move the gaze accordingly, and may continue finding
	new objects and salient points as it does. However, if either an object or
	salient point is selected which has a high Gaussian result (even if it is
	the lowest of all viewable salient points), the mesh will be analysed to find
	a section which has the lowest amount of vertices. This will be repeated ad
	finitum until the user desires to stop mapping.
	*/
}

GazeVisualizer &GazeSolver::vis()
{
	return visualizer;
}

DetectedKeypoints GazeSolver::detectKeypoints()
{
	// Detect the ORB key points in the image
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::ORB> detector = cv::ORB::create();
	detector->detect(visualizer.getImage(), keypoints);
	return mostSalientKeypoints(keypoints);
}

DetectedObjects GazeSolver::detectObjects(const sensor_msgs::Image &img_msg)
{
	std::vector<tf_object_detection::DetectedObject> detected_objects;

	tf_object_detection::ObjectDetection srv;
	srv.request.image = img_msg;
	if (obj_detect_client.call(srv))
	{
		// Only modify the camera image with detected object boxes if detected
		// objects are actually found
		detected_objects = srv.response.detected_objects;
		if (!detected_objects.empty())
		{
			// Update the visualization image
			sensor_msgs::Image result_img_msg = srv.response.result_image;
			visualizer.update(result_img_msg);

			// Report the detected objects
			for (auto &obj : detected_objects)
			{
				ROS_INFO("Class: %s", obj.obj_class.c_str());
				ROS_INFO("Score: %f", obj.score);
				ROS_INFO("Left: %u", obj.left);
				ROS_INFO("Top: %u", obj.top);
				ROS_INFO("Right: %u", obj.right);
				ROS_INFO("Bottom: %u", obj.bottom);
			}
		}
	}
	else
	{
		ROS_WARN("Did not get a response from tf_object_detection service");
	}

	return detected_objects;
}

DetectedKeypoints GazeSolver::mostSalientKeypoints(DetectedKeypoints &keypoints)
{
	// Create the vector containing the wrapped salient keypoints, in order for
	// their saliency scores to be calculated
	std::vector<SalientPoint> salient_points;
	std::for_each(keypoints.begin(), keypoints.end(), [&](cv::KeyPoint& k)
	{
		salient_points.emplace_back(k, k.pt.x, k.pt.y, 0);
	});

	// Calculate the standard deviation of the keypoints, and erase those
	// that are lower than the standard deviation.
	double sd = calculateSaliencySD(salient_points);
	salient_points.erase(std::remove_if(salient_points.begin(), salient_points.end(),
		[&](SalientPoint& p) { return p.getSaliencyScore() < sd; }),
		salient_points.end());

	// Return the most salient points
	std::vector<cv::KeyPoint> output;
	for (const auto& point : salient_points)
		output.push_back(point.getKeyPoint());

	return output;
}