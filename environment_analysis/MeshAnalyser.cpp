#include "MeshAnalyser.hpp"

// Eigen: inverse() definition
#include <Eigen/Dense>

#include <limits>

MeshAnalyser::MeshAnalyser()
{
	
}

std::vector<MeshVertex3D> MeshAnalyser::project(const Eigen::MatrixXf &perspective,
                                                const Eigen::MatrixXf &pose)
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

std::vector<MeshVertex2D> MeshAnalyser::projectToScreen(float width, float height)
{
	std::vector<MeshVertex2D> output;

	float half_width = width / 2.0f;
	float half_height = height / 2.0f;

	std::vector<MeshVertex3D> projected_vertices = project(perspective, pose);
	for (const auto &vertex : projected_vertices)
	{
		// Convert the 3D position to the 2D screen position
		float screen_x = (vertex.x * width) / (2.0f * vertex.w) + half_width;
		float screen_y = (vertex.y * height) / (2.0f * vertex.w) + half_height;
		output.emplace_back(screen_x, screen_y);
	}

	return output;
}

void MeshAnalyser::setPerspective(const Eigen::MatrixXf &perspective)
{
	this->perspective = perspective;
}

void MeshAnalyser::setPose(const Eigen::MatrixXf &pose)
{
	this-> pose = pose;
}

void MeshAnalyser::addVertex(float x, float y, float z)
{
	mesh.emplace_back(x, y, z, 1.0f);
}

void MeshAnalyser::clearMesh()
{
	mesh.clear();
}

Eigen::MatrixXf MeshAnalyser::getRotationAsTransform(float x, float y, float z)
{
	Eigen::MatrixXf x_transform(4,4);
	// Rotation
	x_transform(0,0) = 1;
	x_transform(0,1) = 0;
	x_transform(0,2) = 0;
	x_transform(1,0) = 0;
	x_transform(1,1) = cosf(x);
	x_transform(1,2) = -sinf(x);
	x_transform(2,0) = 0;
	x_transform(2,1) = sinf(x);
	x_transform(2,2) = cosf(x);
	// Translation
	x_transform(0,3) = 0;
	x_transform(1,3) = 0;
	x_transform(2,3) = 0;
	// Constant
	x_transform(3,0) = 0;
	x_transform(3,1) = 0;
	x_transform(3,2) = 0;
	x_transform(3,3) = 1;

	Eigen::MatrixXf y_transform(4,4);
	// Rotation
	y_transform(0,0) = cosf(y);
	y_transform(0,1) = 0;
	y_transform(0,2) = sinf(y);
	y_transform(1,0) = 0;
	y_transform(1,1) = 1;
	y_transform(1,2) = 0;
	y_transform(2,0) = -sinf(y);
	y_transform(2,1) = 0;
	y_transform(2,2) = cosf(y);
	// Translation
	y_transform(0,3) = 0;
	y_transform(1,3) = 0;
	y_transform(2,3) = 0;
	// Constant
	y_transform(3,0) = 0;
	y_transform(3,1) = 0;
	y_transform(3,2) = 0;
	y_transform(3,3) = 1;

	Eigen::MatrixXf z_transform(4,4);
	// Rotation
	z_transform(0,0) = cosf(z);
	z_transform(0,1) = -sinf(z);
	z_transform(0,2) = 0;
	z_transform(1,0) = sinf(z);
	z_transform(1,1) = cosf(z);
	z_transform(1,2) = 0;
	z_transform(2,0) = 0;
	z_transform(2,1) = 0;
	z_transform(2,2) = 1;
	// Translation
	z_transform(0,3) = 0;
	z_transform(1,3) = 0;
	z_transform(2,3) = 0;
	// Constant
	z_transform(3,0) = 0;
	z_transform(3,1) = 0;
	z_transform(3,2) = 0;
	z_transform(3,3) = 1;

	return (x_transform * y_transform * z_transform);
}

Rotation MeshAnalyser::findLesserMappedSection()
{
	// Look at the entire mesh by continuously rotating the pose matrix so that
	// every possible viewing angle has been achieved. A lesser-mapped section
	// of the mesh will be one that has the fewest vertices.

	// Analysis of the mesh can be viewed as simply looking outwards at all
	// possible points on a sphere (with the sphere representing the camera and
	// being infinitely small). Since this equates to a large number of viewing
	// points, an angular step is used to provide a fast way of performing this
	// search.

	size_t least_vertices = std::numeric_limits<unsigned int>::max();
	Rotation angles;

	const float FULL_CIRCLE = 2.0f * 3.14159654f;
	const int STEP = 4;
	const float ANGULAR_STEP = FULL_CIRCLE / (float)STEP;

	// The outer loop only needs to go halfway around the sphere, as each inner
	// loop does a full rotation of the circumference
	for (int x = 0; x < (STEP / 2); x++)
	{
		// Do a full loop of the circumference of the sphere
		for (int y = 0; y < STEP; y++)
		{
			float x_angle = x * ANGULAR_STEP;
			float y_angle = y * ANGULAR_STEP;
			Eigen::MatrixXf rot_pose = pose * getRotationAsTransform(x_angle, y_angle, 0.0f);

			size_t current_vertices = project(perspective, rot_pose).size();
			if (current_vertices < least_vertices)
			{
				least_vertices = current_vertices;
				angles.x = ANGULAR_STEP * x;
				angles.y = ANGULAR_STEP * y;
				angles.z = 0.0f;
			}
		}
	}

	return angles;
}