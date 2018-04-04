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

	SphericalMapper(float fov_horiz, float fov_vert);

	GazePoint next();
	bool hasNext();

private:
	size_t current_gaze_index;
	float fov_horiz, fov_vert;
	std::queue<GazePoint> gaze_points;

	std::queue<GazePoint> createGazeSphere(float fov_horiz, float fov_vert, float radius);
};

#endif // _SPHERICAL_MAPPER_H_