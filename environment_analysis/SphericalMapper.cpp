#include "SphericalMapper.hpp"

#include <assert.h>

#include "ros/ros.h"

SphericalMapper::SphericalMapper(float diag_fov)
{
	this->current_gaze_index = 0;
	this->diag_fov = diag_fov;

	gaze_points = createGazeSphere(diag_fov, 5.0f);
}

SphericalMapper::GazePoint SphericalMapper::next()
{
	SphericalMapper::GazePoint point = gaze_points.front();
	gaze_points.pop();
	return point;
}

bool SphericalMapper::hasNext()
{
	return !gaze_points.empty();
}

// Computes the length of the chord using a percentage of the total radius
float chordLength(float norm_dist_along_radius, float radius = 1.0f)
{
	assert(norm_dist_along_radius >= -1.0f && norm_dist_along_radius <= 1.0f);

	const float half_circle = 3.141592654f;
	const float quarter_circle = 0.5f * half_circle;

	float theta = quarter_circle * norm_dist_along_radius;
	// Prevent a divide by zero
	if (sinf(theta) != 0.0f)
		return (radius / sinf(theta)) * sinf(half_circle - (2 * theta));
	else
		return 2.0f * radius;
}

std::queue<SphericalMapper::GazePoint> SphericalMapper::createGazeSphere(float diag_fov,
                                                                         float radius = 1.0f)
{
	// Define all the points that make up a sphere using a step interval.
	std::queue<SphericalMapper::GazePoint> points;

	const float horiz_step = diag_fov;
	const float vert_step = diag_fov;
	const float full_circle = 2.0f * 3.141592654f;
	const float half_circle = 3.141592654f;
	const float quarter_circle = 0.5f * half_circle;

	// The sine of the interval values between -1/4 and 1/4 circle will give
	// results between -1 and 1, which will be used to define the normalised
	// distance along the radius (-1 is the bottom of the sphere and 1 is the
	// top).
	for (float h = 0; h < quarter_circle; h += vert_step)
	{
		// The y-coordinate determines how far the normalised distance is from
		// the sphere's center, hence it can be used to determine the chord
		// length (and radius) at any distance.
		float y = sinf(h);
		float slice_radius = chordLength(y) * 0.5f;
		ROS_INFO("===== Slice at %.3f from the center (0.0f) (RADIUS: %.3f) =====", y, slice_radius);
		for (float w = 0.0f; w < full_circle; w += horiz_step)
		{
			// This is a RHS-with-Y-up coordinate system, so w will change x/z
			// whilst h will change the y coordinate
			float x = cosf(w) * slice_radius;
			float z = sinf(w) * slice_radius;

			// Multiply by the desired sphere radius and store it with the other
			// points
			points.push({x * radius, y * radius, z * radius});
			ROS_INFO("Point at %.3f: (%.3f,%.3f,%.3f)", w, points.back().x, points.back().y, points.back().z);
		}
	}

	return points;
}