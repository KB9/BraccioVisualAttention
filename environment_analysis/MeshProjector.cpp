#include "MeshProjector.hpp"

// Eigen: inverse() definition
#include <Eigen/Dense>

MeshProjector::MeshProjector()
{
	
}

std::vector<MeshVertex2D> MeshProjector::project()
{
	std::vector<MeshVertex2D> output;

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
		v(3,0) = 1;

		// Apply the perspective projection to the vertex position
		auto result = projection * v;
		float x = result(0,0);
		float y = result(1,0);
		float w = result(3,0);

		// Convert the 3D position to 2D screen positions. Ensure that the positions
		// are in front of the camera (i.e. w >= 0.0f) to avoid mapping points that
		// are behind the camera onto the projection.
		if (w >= 0.0f)
		{
			float screen_x = (x * 1280.0f) / (2.0f * w) + 640.0f;
			float screen_y = (y * 720.0f) / (2.0f * w) + 360.0f;
			output.emplace_back(screen_x, screen_y);
		}
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
	mesh.emplace_back(x, y, z);
}

void MeshProjector::clearMesh()
{
	mesh.clear();
}