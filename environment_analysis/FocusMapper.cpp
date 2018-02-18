#include "FocusMapper.hpp"

#include <cmath>

FocusMapper::FocusMapper()
{

}

void FocusMapper::add(float x, float y, float z, float radius)
{
	points.emplace_back(x, y, z, radius);
}

float FocusMapper::calculate(float x, float y, float z)
{
	float result = 0.0f;
	for (const auto &point : points)
	{
		float x_dist = x - point.x;
		float y_dist = y - point.y;
		float z_dist = z - point.z;
		float magnitude = std::sqrt(std::pow(x_dist,2) + std::pow(y_dist,2) + std::pow(z_dist,2));

		if (magnitude < point.radius)
		{
			result += (magnitude / point.radius);
		}
	}
	return result;
}

// TODO: Could make it decay faster based on how many times it has been decayed
// before (i.e. older focus points decay at a faster rate than newer ones).
void FocusMapper::decay(float rate)
{
	for (size_t i = 0; i < points.size(); i++)
	{
		points[i].radius -= rate;
		if (points[i].radius <= 0)
		{
			points.erase(points.begin() + i);
		}
	}
}

void FocusMapper::clear()
{
	points.clear();
}