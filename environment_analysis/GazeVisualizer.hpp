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
	void drawKeyPoint(const cv::KeyPoint &point);
	void setGazePoint(int x, int y);

	void update(const sensor_msgs::Image &img_msg);
	void show();

	// For direct access to the image matrix
	cv::Mat &getImage();

private:
	cv::Mat image;
	cv::Mat gaze_target_image;
};

#endif