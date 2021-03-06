#ifndef _ZED_SPATIAL_MAPPER_H_
#define _ZED_SPATIAL_MAPPER_H_

#include <chrono>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <sl/Camera.hpp>

#include "zed_wrapper/Mesh.h"

class ZedSpatialMapper
{
public:
	ZedSpatialMapper(std::shared_ptr<sl::Camera> zed);

	void start();
	void stop();
	bool save(const std::string &filename);

	void update();

	void publish(ros::Publisher &pub_mesh);

private:
	std::shared_ptr<sl::Camera> zed;

	sl::Mat left_image; // sl::Mat to hold images
	sl::Pose pose;      // sl::Pose to hold pose data
	sl::TRACKING_STATE tracking_state;

	bool is_mapping = false;
	sl::Mesh mesh;
	sl::SpatialMappingParameters spatial_mapping_params;
	sl::MeshFilterParameters filter_params;

	std::chrono::high_resolution_clock::time_point t_last;

	zed_wrapper::Mesh mesh_msg;

	sl::Transform perspective;

	void updateMsg();

	sl::Transform createPerspective();
};

#endif // _ZED_SPATIAL_MAPPER_H_