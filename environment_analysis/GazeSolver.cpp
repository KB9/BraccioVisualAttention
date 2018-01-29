#include "GazeSolver.hpp"

// Infra
#include <limits>
#include <utility>

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

GazeSolver::GazeSolver(const ros::ServiceClient &obj_detect_client,
                       float diag_fov)
{
	this->obj_detect_client = obj_detect_client;
	this->diag_fov = diag_fov;
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
		visualizer.drawKeyPoint(kp.getKeyPoint());

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

	// Convert the PCL ROS msg into a PointCloud before pulling points from it
	PointCloud cloud;
	pcl::fromROSMsg(*(data.cloud), cloud);

	gaussian_map.decay(5.0f);

	// Try to look at any objects first
	// TODO: Don't look at the object if the Gaussian result is too high
	while (!objects.empty())
	{
		auto screen_pos = toScreen(objects[0]);
		GazePoint gaze_point = find3dPoint(screen_pos, data, cloud);

		ROS_INFO("Focusing gaze on detected object.");
		gaussian_map.add(gaze_point.x, gaze_point.y, gaze_point.z, 1000);
		visualizer.setGazePoint(screen_pos.x, screen_pos.y);
		if (gaze_point.is_estimate) ROS_WARN("Using estimated gaze point!");
		return gaze_point;
	}

	// Then try to look at the best salient keypoint next
	if (!keypoints.empty())
	{
		std::pair<SalientPoint, GazePoint> best = findBestKeyPoint(keypoints, data, cloud);
		SalientPoint keypoint = best.first;
		GazePoint gaze_point = best.second;

		ROS_INFO("Focusing gaze on detected salient point.");
		gaussian_map.add(gaze_point.x, gaze_point.y, gaze_point.z, 1000);
		visualizer.setGazePoint(keypoint.getCameraX(), keypoint.getCameraY());
		if (gaze_point.is_estimate) ROS_WARN("Using estimated gaze point!");
		return gaze_point;
	}

	// If neither objects or keypoints could be looked at, attempt to look
	// at an under-mapped area of the environment mesh
	ROS_INFO("Focusing gaze on under-mapped mesh section.");

	// Set the perspective and pose matrices used to project the mesh vertices
	const Eigen::MatrixXf perspective = toEigenMatrix(data.mesh.perspective);
	const Eigen::MatrixXf pose = toEigenMatrix(data.mesh.pose);

	// Find the angles required to view an undermapped section of the mesh
	// Use a fake distant point and rotate it according to the mesh analysis
	// angles to focus gaze on it
	// NOTE: Mesh analysis takes a REALLY long time
	GazePoint fake_point{0.0f, 0.0f, 5.0f, true};
	Rotation rot = mesh_analyser.findLesserMappedSection(data.mesh,
	                                                     perspective,
	                                                     pose);
	return rotate3dPoint(fake_point, rot.x, rot.y, rot.z);
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
		visualizer.drawKeyPoint(kp.getKeyPoint());

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
		//ROS_WARN("Did not get a response from tf_object_detection service");
	}

	return detected_objects;
}

DetectedKeypoints GazeSolver::mostSalientKeypoints(std::vector<cv::KeyPoint> &keypoints)
{
	// Create the vector containing the wrapped salient keypoints, in order for
	// their saliency scores to be calculated
	DetectedKeypoints salient_points;
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

	return salient_points;
}

std::pair<SalientPoint, GazePoint> GazeSolver::findBestKeyPoint(const DetectedKeypoints &keypoints,
                                                                const SensorData &data,
                                                                const PointCloud &cloud)
{
	float max_saliency = std::numeric_limits<float>::min();
	float min_gaussian = std::numeric_limits<float>::max();
	SalientPoint best_keypoint = keypoints[0];
	GazePoint best_gaze_point;
	for (const auto &keypoint : keypoints)
	{
		auto screen_pos = toScreen(keypoint.getKeyPoint());
		GazePoint gaze_point = find3dPoint(screen_pos, data, cloud);
		float saliency = keypoint.getSaliencyScore();
		float gaussian = gaussian_map.calculate(gaze_point.x, gaze_point.y, gaze_point.z);
		if (saliency >= max_saliency && gaussian <= min_gaussian)
		{
			best_keypoint = keypoint;
			best_gaze_point = gaze_point;
			max_saliency = saliency;
			min_gaussian = gaussian;
		}
	}
	ROS_INFO("Salient point info: SALIENCY=%.3f, GAUSSIAN=%.3f", max_saliency, min_gaussian);
	return {best_keypoint, best_gaze_point};
}

GazePoint GazeSolver::find3dPoint(const ScreenPosition &screen,
                                  const SensorData &data,
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

	return {pcl_x, pcl_y, pcl_z, false};
}

ScreenPosition GazeSolver::toScreen(const tf_object_detection::DetectedObject &object)
{
	// Determine the center point of the bounding box containing the object
	unsigned width = object.right - object.left;
	unsigned height = object.bottom - object.top;
	unsigned center_x = object.left + (width / 2);
	unsigned center_y = object.top + (height / 2);

	return {center_x, center_y};
}

ScreenPosition GazeSolver::toScreen(const cv::KeyPoint &keypoint)
{
	return {keypoint.pt.x, keypoint.pt.y};
}

GazePoint GazeSolver::createFakePoint(const ScreenPosition &screen,
                                      float diag_fov,
                                      unsigned screen_width, unsigned screen_height)
{
	float diag_length = std::sqrt(std::pow(screen_width,2) + std::pow(screen_height,2));
	float rads_per_pixel = diag_fov / diag_length;

	unsigned screen_cx = screen_width / 2;
	unsigned screen_cy = screen_height / 2;

	float angle_x = (screen_cx - screen.x) * rads_per_pixel;
	float angle_y = (screen_cy - screen.y) * rads_per_pixel;

	GazePoint fake_point;
	fake_point.z = 20.0f;
	fake_point.x = fake_point.z * tanf(angle_x);
	fake_point.y = fake_point.z * tanf(angle_y);
	fake_point.is_estimate = true;

	return fake_point;
}

GazePoint GazeSolver::rotate3dPoint(const GazePoint &point,
                                    float x_angle, float y_angle, float z_angle)
{
	Eigen::MatrixXf x_rotate(3,3);
	x_rotate(0,0) = 1;
	x_rotate(0,1) = 0;
	x_rotate(0,2) = 0;
	x_rotate(1,0) = 0;
	x_rotate(1,1) = cosf(x_angle);
	x_rotate(1,2) = -sinf(x_angle);
	x_rotate(2,0) = 0;
	x_rotate(2,1) = sinf(x_angle);
	x_rotate(2,2) = cosf(x_angle);

	Eigen::MatrixXf y_rotate(3,3);
	y_rotate(0,0) = cosf(y_angle);
	y_rotate(0,1) = 0;
	y_rotate(0,2) = sinf(y_angle);
	y_rotate(1,0) = 0;
	y_rotate(1,1) = 1;
	y_rotate(1,2) = 0;
	y_rotate(2,0) = -sinf(y_angle);
	y_rotate(2,1) = 0;
	y_rotate(2,2) = cosf(y_angle);

	Eigen::MatrixXf z_rotate(3,3);
	z_rotate(0,0) = cosf(z_angle);
	z_rotate(0,1) = -sinf(z_angle);
	z_rotate(0,2) = 0;
	z_rotate(1,0) = sinf(z_angle);
	z_rotate(1,1) = cosf(z_angle);
	z_rotate(1,2) = 0;
	z_rotate(2,0) = 0;
	z_rotate(2,1) = 0;
	z_rotate(2,2) = 1;

	Eigen::MatrixXf pos(3,1);
	pos(0,0) = point.x;
	pos(1,0) = point.y;
	pos(2,0) = point.z;

	Eigen::MatrixXf result = (x_rotate * (y_rotate * (z_rotate * pos)));
	return {result(0,0), result(1,0), result(2,0)};
}