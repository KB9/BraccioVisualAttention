#ifndef _MESH_PROJECTOR_H_
#define _MESH_PROJECTOR_H_

// Infra
#include <vector>

// Eigen
#include <Eigen/Core>

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

struct ProjectionOptions
{
	float x_angle = 0.0f;
	float y_angle = 0.0f;
	float z_angle = 0.0f;
};

class MeshProjector
{
public:
	MeshProjector();

	std::vector<MeshVertex3D> project();
	std::vector<MeshVertex2D> projectToScreen(float width, float height);

	void setPerspective(const Eigen::MatrixXf &camera);
	void setPose(const Eigen::MatrixXf &pose);

	void addVertex(float x, float y, float z);
	void clearMesh();

	void rotatePose(float x, float y, float z);

private:
	std::vector<MeshVertex3D> mesh;

	Eigen::MatrixXf perspective;
	Eigen::MatrixXf pose;
};

#endif // _MESH_PROJECTOR_H_