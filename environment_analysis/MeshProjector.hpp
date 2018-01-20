#ifndef _MESH_PROJECTOR_H_
#define _MESH_PROJECTOR_H_

// Infra
#include <vector>

// Eigen
#include <Eigen/Core>

struct MeshVertex3D
{
	MeshVertex3D(float x, float y, float z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	float x, y, z;
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

class MeshProjector
{
public:
	MeshProjector();

	std::vector<MeshVertex2D> project();

	void setPerspective(const Eigen::MatrixXf &camera);
	void setPose(const Eigen::MatrixXf &pose);

	void addVertex(float x, float y, float z);
	void clearMesh();

private:
	std::vector<MeshVertex3D> mesh;

	Eigen::MatrixXf perspective;
	Eigen::MatrixXf pose;
};

#endif // _MESH_PROJECTOR_H_