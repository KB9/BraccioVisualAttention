#include "SalientPoint.hpp"

SalientPoint::SalientPoint(const cv::KeyPoint& keypoint,
	double camera_x, double camera_y, double camera_z)
{
	this->keypoint = keypoint;
	this->camera_x = camera_x;
	this->camera_y = camera_y;
	this->camera_z = camera_z;
	saliency_score = calculateSaliencyScore(keypoint);
}

double SalientPoint::getSaliencyScore() const
{
	return saliency_score;
}

const cv::KeyPoint& SalientPoint::getKeyPoint() const
{
	return keypoint;
}

double SalientPoint::getCameraX() const
{
	return camera_x;
}

double SalientPoint::getCameraY() const
{
	return camera_y;
}

double SalientPoint::getCameraZ() const
{
	return camera_z;
}

bool SalientPoint::wasAttended() const
{
	return attended;
}

double SalientPoint::calculateSaliencyScore(const cv::KeyPoint& keypoint)
{
	const int x = keypoint.pt.x;
	const int y = keypoint.pt.y;
	const int delta = keypoint.octave;
	const double k = 0.01;

	return x * y * delta * k;
}