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

GazePoint GazeSolver::next(const SensorData &data)
{
	// Update the gaze visualizer with the latest image from the camera
	visualizer.update(data.image);

	// Detect the keypoints and objects in the image (keypoint detection must
	// occur before objects, as the object detection will update the vis image)
	DetectedKeypoints keypoints = detectKeypoints();
	DetectedObjects objects = detectObjects(data.image);

	// Draw the keypoints on the image
	for (const auto &kp : keypoints)
		visualizer.drawKeyPoint(kp);

	// Set the perspective and pose matrices used to project the mesh vertices
	const Eigen::MatrixXf perspective = toEigenMatrix(data.mesh.perspective);
	const Eigen::MatrixXf pose = toEigenMatrix(data.mesh.pose);
	mesh_analyser.setPerspective(perspective);
	mesh_analyser.setPose(pose);

	// Register all mesh vertices with the mesh analyser
	mesh_analyser.clearMesh();
	for (auto &chunk : data.mesh.chunks)
	{
		for (auto &v : chunk.vertices)
		{
			mesh_analyser.addVertex(v.x, v.y, v.z);
		}
	}

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

	// Convert the point cloud ROS message to a PointCloud object
	PointCloud cloud;
	if (data.cloud == nullptr) return {0.0f, 0.0f, 0.0f}; // TODO: TEMPORARY FIX
	pcl::fromROSMsg(*(data.cloud), cloud);

	// Look at objects first
	if (!objects.empty())
	{
		ROS_INFO("Focusing gaze on detected object.");
		return find3dPoint(objects[0], cloud);
	}
	// Look at keypoints if no objects are found
	else if (!keypoints.empty())
	{
		ROS_INFO("Focusing gaze on detected salient keypoint.");
		return find3dPoint(keypoints[0], cloud);
	}
	// If there are no objects/keypoints, find an under-mapped section of the
	// mesh to look at
	else
	{
		ROS_INFO("Focusing gaze on under-mapped mesh section.");
		// Find the angles required to view an undermapped section of the mesh
		// NOTE: This currently takes a REALLY long time, so disabled for now
		// Rotation rot = mesh_analyser.findLesserMappedSection();
		// ROS_INFO("Required: (%.3f,%.3f,%.3f)", rot.x, rot.y, rot.z);
		return {0.0f, 0.0f, 0.0f}; // TODO: Make this return a point
	}
}

void GazeSolver::showVisualization(const sensor_msgs::Image &img_msg)
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

	visualizer.show();
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

GazePoint GazeSolver::find3dPoint(unsigned screen_x, unsigned screen_y,
                                  PointCloud &cloud)
{
	// If the cloud isn't organized, the PCL point can't be determined
	if (!cloud.isOrganized())
	{
		ROS_WARN("Point cloud not organized - can't find point for (%u,%u)", screen_x, screen_y);
		return {0.0f, 0.0f, 0.0f};
	}

	// If the PCL point isn't finite, the 3D position can't be calculated
	pcl::PointXYZRGB point = cloud(screen_x, screen_y);
	if (!pcl::isFinite(point))
	{
		ROS_WARN("PCL point is not finite - cannot calculate 3D position for (%u,%u)", screen_x, screen_y);
		return {0.0f, 0.0f, 0.0f};
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

	// Update the visualizer with the selected gaze point
	visualizer.setGazePoint(screen_x, screen_y);

	return {pcl_x, pcl_y, pcl_z};
}

GazePoint GazeSolver::find3dPoint(tf_object_detection::DetectedObject object,
                                  PointCloud &cloud)
{
	// Determine the center point of the bounding box containing the object
	unsigned width = object.right - object.left;
	unsigned height = object.bottom - object.top;
	unsigned center_x = object.left + (width / 2);
	unsigned center_y = object.top + (height / 2);

	return find3dPoint(center_x, center_y, cloud);
}

GazePoint GazeSolver::find3dPoint(cv::KeyPoint keypoint,
                                  PointCloud &cloud)
{
	return find3dPoint(keypoint.pt.x, keypoint.pt.y, cloud);
}

GazePoint GazeSolver::alignPoint(const GazePoint &point,
                                 float effector_x_angle, float effector_y_angle,
                                 float effector_z_angle)
{
	// Eigen::MatrixXf x_rotate(3,3);
	// x_rotate(0,0) = 1;
	// x_rotate(0,1) = 0;
	// x_rotate(0,2) = 0;
	// x_rotate(1,0) = 0;
	// x_rotate(1,1) = cosf(x_angle);
	// x_rotate(1,2) = -sinf(x_angle);
	// x_rotate(2,0) = 0;
	// x_rotate(2,1) = sinf(x_angle);
	// x_rotate(2,2) = cosf(x_angle);

	// Eigen::MatrixXf y_rotate(3,3);
	// y_rotate(0,0) = cosf(y_angle);
	// y_rotate(0,1) = 0;
	// y_rotate(0,2) = sinf(y_angle);
	// y_rotate(1,0) = 0;
	// y_rotate(1,1) = 1;
	// y_rotate(1,2) = 0;
	// y_rotate(2,0) = -sinf(y_angle);
	// y_rotate(2,1) = 0;
	// y_rotate(2,2) = cosf(y_angle);

	// Eigen::MatrixXf z_rotate(3,3);
	// z_rotate(0,0) = cosf(z_angle);
	// z_rotate(0,1) = -sinf(z_angle);
	// z_rotate(0,2) = 0;
	// z_rotate(1,0) = sinf(z_angle);
	// z_rotate(1,1) = cosf(z_angle);
	// z_rotate(1,2) = 0;
	// z_rotate(2,0) = 0;
	// z_rotate(2,1) = 0;
	// z_rotate(2,2) = 1;

	Eigen::MatrixXf x_rotate(3,3);
	x_rotate(0,0) = 1;
	x_rotate(0,1) = 0;
	x_rotate(0,2) = 0;
	x_rotate(1,0) = 0;
	x_rotate(1,1) = cosf(effector_x_angle);
	x_rotate(1,2) = sinf(effector_x_angle);
	x_rotate(2,0) = 0;
	x_rotate(2,1) = -sinf(effector_x_angle);
	x_rotate(2,2) = cosf(effector_x_angle);

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
	pos(0,0) = point.x;
	pos(1,0) = point.y;
	pos(2,0) = point.z;

	Eigen::MatrixXf result = (x_rotate * y_rotate * z_rotate) * pos;
	float x = result(0,0);
	float y = result(1,0);
	float z = result(2,0);
	return {x, y, z};
}