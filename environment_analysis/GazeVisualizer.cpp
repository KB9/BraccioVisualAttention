#include "GazeVisualizer.hpp"

// OpenCV
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

void GazeVisualizer::drawKeyPoint(const cv::KeyPoint &point)
{
	cv::drawKeypoints(image, { point }, image);
}

void GazeVisualizer::setGazePoint(int x, int y)
{
	// Clone the last image that was shown and draw the gaze point on it
	cv::Mat temp = image.clone();
	cv::circle(temp, {x, y}, 20, {0,255,0,255}, 5);

	// Shrink the gaze target image so it doesn't obscure the normal image
	// when combined later
	gaze_target_image = cv::Mat((int)(temp.rows * 0.25), (int)(temp.cols * 0.25), temp.type());
	cv::resize(temp, gaze_target_image, cv::Size(), 0.25, 0.25);
}

void GazeVisualizer::update(const sensor_msgs::Image &img_msg)
{
	image = image = cv_bridge::toCvCopy(img_msg)->image;
}

void GazeVisualizer::show()
{
	// Create a temporary image
	cv::Mat temp = image.clone();

	// Combine the gaze target image and normal image, if there is a gaze target
	if (!gaze_target_image.empty())
		gaze_target_image.copyTo(temp(cv::Rect(0, 0, gaze_target_image.cols, gaze_target_image.rows)));

	// Draw red line overlay to indicate center of image
	cv::line(temp, {temp.cols/2,0}, {temp.cols/2,temp.rows}, {0,0,255,255});
	cv::line(temp, {0,temp.rows/2}, {temp.cols,temp.rows/2}, {0,0,255,255});

	// Display the image
	cv::imshow("view", temp);
	cv::waitKey(30);
}

cv::Mat &GazeVisualizer::getImage()
{
	return image;
}