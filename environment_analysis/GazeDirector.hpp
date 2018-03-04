#ifndef _GAZE_DIRECTOR_H_
#define _GAZE_DIRECTOR_H_

// Infra
#include <utility>

// ROS
#include "ros/ros.h"

#include "SphericalMapper.hpp"
#include "SceneAnalyzer.hpp"

class GazeDirector
{
public:
	enum class PointType
	{
		NONE,
		SCENE,
		SALIENT
	};

	struct GazePoint
	{
		float x, y, z;
		PointType type;
	};

	GazeDirector(const ros::ServiceClient &obj_detect_client,
	             std::function<SceneAnalyzer::ScenePoint(const SceneAnalyzer::ScenePoint &camera_point)> camera_to_world,
	             float diag_fov);

	GazePoint next(const SceneAnalyzer::SceneData &data);
	bool hasNext();

	void visualize(const SceneAnalyzer::SceneData &data);

private:
	SphericalMapper spherical_mapper;
	SceneAnalyzer scene_analyzer;
	bool analyze_scene;
};

#endif // _GAZE_DIRECTOR_H_