#include "GazeDirector.hpp"

// TESTING
#include <thread>
#include <chrono>

SceneAnalyzer::ScenePoint toBraccioKinematicsAxes(const SceneAnalyzer::ScenePoint &point)
{
	return {point.z * 100.0f, -point.x * 100.0f, point.y * 100.0f};
}

GazeDirector::GazeDirector(const ros::ServiceClient &obj_detect_client,
                           std::function<SceneAnalyzer::ScenePoint(const SceneAnalyzer::ScenePoint &camera_point)> camera_to_world,
                           float diag_fov) :
	spherical_mapper(diag_fov),
	scene_analyzer(obj_detect_client, camera_to_world, diag_fov),
	analyze_scene(false)
{

}

GazeDirector::GazePoint GazeDirector::next(const SceneAnalyzer::SceneData &data)
{
	// Analyze the current scene if the Braccio has been set to look at a new
	// scene
	if (analyze_scene)
	{
		scene_analyzer.analyze(data);
		analyze_scene = false;
	}

	// If there are still points in the scene that need to be attended to, do
	// those first
	if (scene_analyzer.hasNext())
	{
		SceneAnalyzer::ScenePoint point = scene_analyzer.next();
		return {point.x, point.y, point.z, PointType::SALIENT};
	}
	// If there are no more points in the scene to be attended to, look at a new
	// scene and find interesting points within it
	else if (spherical_mapper.hasNext())
	{
		SphericalMapper::GazePoint gaze_point = spherical_mapper.next();
		auto braccio_point = toBraccioKinematicsAxes({gaze_point.x, gaze_point.y, gaze_point.z});
		analyze_scene = true;
		return {braccio_point.x, braccio_point.y, braccio_point.z, PointType::SCENE};
	}
	else
	{
		// TODO: Do something better than this
		return {0, 0, 0, PointType::NONE};
	}
}

bool GazeDirector::hasNext()
{
	return spherical_mapper.hasNext() || scene_analyzer.hasNext();
}

void GazeDirector::visualize(const SceneAnalyzer::SceneData &data)
{
	scene_analyzer.visualize(data);
}