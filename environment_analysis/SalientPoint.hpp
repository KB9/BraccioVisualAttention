#ifndef _SALIENTPOINT_HPP_
#define _SALIENTPOINT_HPP_

#include "opencv2/core.hpp"

class SalientPoint
{
public:
	SalientPoint(const cv::KeyPoint& keypoint,
		double camera_x, double camera_y, double camera_z);

	double getSaliencyScore() const;
	const cv::KeyPoint& getKeyPoint() const;
	double getCameraX() const;
	double getCameraY() const;
	double getCameraZ() const;
	bool wasAttended() const; 

private:
	static double calculateSaliencyScore(const cv::KeyPoint& keypoint);

	double saliency_score;
	cv::KeyPoint keypoint;
	double camera_x, camera_y, camera_z;
	bool attended;
};

#endif