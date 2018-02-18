#ifndef _SPHERICAL_MAPPER_H_
#define _SPHERICAL_MAPPER_H_

#include <vector>
#include <queue>

#include "zed_wrapper/Matrix4f.h"

class SphericalMapper
{
public:
	struct GazePoint
	{
		float x, y, z;
	};

	SphericalMapper(float diag_fov);

	GazePoint next();
	bool hasNext();

private:
	size_t current_gaze_index;
	float diag_fov;
	std::queue<GazePoint> gaze_points;

	std::queue<GazePoint> createGazeSphere(float diag_fov, float radius);
};

#endif // _SPHERICAL_MAPPER_H_