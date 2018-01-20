#include "MeshProjector.hpp"

// Eigen: inverse() definition
#include <Eigen/Dense>

MeshProjector::MeshProjector()
{
	
}

std::vector<MeshVertex3D> MeshProjector::project()
{
	std::vector<MeshVertex3D> output;

	// Calculate the 4x4 projection matrix from the camera and pose matrices
	auto projection = perspective * pose.inverse();

	for (const auto &vertex : mesh)
	{
		// Create a 4x1 matrix which represents the current vertex's position in
		// 3D space
		Eigen::MatrixXf v(4,1);
		v(0,0) = vertex.x;
		v(1,0) = vertex.y;
		v(2,0) = vertex.z;
		v(3,0) = vertex.w;

		// Apply the perspective projection to the vertex position
		auto result = projection * v;
		float x = result(0,0);
		float y = result(1,0);
		float z = result(2,0);
		float w = result(3,0);

		// Ensure that the positions are in front of the camera (i.e. w >= 0.0f)
		// to avoid finding points that are behind the camera.
		if (w >= 0.0f)
		{
			output.emplace_back(x, y, z, w);
		}
	}

	return output;
}

std::vector<MeshVertex2D> MeshProjector::projectToScreen(float width, float height)
{
	std::vector<MeshVertex2D> output;

	float half_width = width / 2.0f;
	float half_height = height / 2.0f;

	std::vector<MeshVertex3D> projected_vertices = project();
	for (const auto &vertex : projected_vertices)
	{
		// Convert the 3D position to the 2D screen position
		float screen_x = (vertex.x * width) / (2.0f * vertex.w) + half_width;
		float screen_y = (vertex.y * height) / (2.0f * vertex.w) + half_height;
		output.emplace_back(screen_x, screen_y);
	}

	return output;
}

void MeshProjector::setPerspective(const Eigen::MatrixXf &perspective)
{
	this->perspective = perspective;
}

void MeshProjector::setPose(const Eigen::MatrixXf &pose)
{
	this-> pose = pose;
}

void MeshProjector::addVertex(float x, float y, float z)
{
	mesh.emplace_back(x, y, z, 1.0f);
}

void MeshProjector::clearMesh()
{
	mesh.clear();
}