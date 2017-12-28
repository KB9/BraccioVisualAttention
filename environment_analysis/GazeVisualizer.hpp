#ifndef _GAZEVISUALIZER_H_
#define _GAZEVISUALIZER_H_

// Infra
#include <vector>

// Image
#include "sensor_msgs/Image.h"

// OpenCV
#include "opencv2/core.hpp"

class GazeVisualizer
{
public:
	GazeVisualizer(const sensor_msgs::ImageConstPtr &img_msg);

	std::vector<cv::KeyPoint> detectKeyPoints();

	void drawKeyPoint(const cv::KeyPoint &point);
	void setGazePoint(int x, int y);

	void update(const sensor_msgs::ImageConstPtr &img_msg);
	void show();

private:
	cv::Mat image;
	cv::Mat gaze_target_image;
};

#endif