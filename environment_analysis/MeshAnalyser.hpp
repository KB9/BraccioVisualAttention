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

struct Rotation
{
	float x, y, z;
};

class MeshAnalyser
{
public:
	MeshAnalyser();

	std::vector<MeshVertex2D> projectToScreen(float width, float height);

	void setPerspective(const Eigen::MatrixXf &camera);
	void setPose(const Eigen::MatrixXf &pose);

	void addVertex(float x, float y, float z);
	void clearMesh();

	Rotation findLesserMappedSection();

private:
	std::vector<MeshVertex3D> mesh;
	Eigen::MatrixXf perspective;
	Eigen::MatrixXf pose;

	std::vector<MeshVertex3D> project(const Eigen::MatrixXf &perspective,
	                                  const Eigen::MatrixXf &pose);

	Eigen::MatrixXf getRotationAsTransform(float x, float y, float z);
};

#endif // _MESH_PROJECTOR_H_