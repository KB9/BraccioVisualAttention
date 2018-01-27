#ifndef _MESH_PROJECTOR_H_
#define _MESH_PROJECTOR_H_

// Infra
#include <vector>

// Eigen
#include <Eigen/Core>

// ZED mesh
#include "zed_wrapper/Mesh.h"

struct MeshVertex3D
{
	MeshVertex3D(float x, float y, float z, float w)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
	}

	float x, y, z, w;
};

struct MeshVertex2D
{
	MeshVertex2D(float x, float y)
	{
		this->x = x;
		this->y = y;
	}

	float x, y;
};

struct Rotation
{
	float x, y, z;
};

class MeshAnalyser
{
public:
	MeshAnalyser();

	std::vector<MeshVertex2D> projectToScreen(const zed_wrapper::Mesh &mesh,
	                                          const Eigen::MatrixXf &perspective,
	                                          const Eigen::MatrixXf &pose,
	                                          float width, float height);

	Rotation findLesserMappedSection(const zed_wrapper::Mesh &mesh,
	                                 const Eigen::MatrixXf &perspective,
	                                 const Eigen::MatrixXf &pose);

private:
	std::vector<MeshVertex3D> project(const zed_wrapper::Mesh &mesh,
	                                  const Eigen::MatrixXf &perspective,
	                                  const Eigen::MatrixXf &pose);

	Eigen::MatrixXf getRotationAsTransform(float x, float y, float z);
};

#endif // _MESH_PROJECTOR_H_