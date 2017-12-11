#include "GaussianMap.hpp"

#include <cmath>

GaussianMap::GaussianMap()
{

}

void GaussianMap::add(float x, float y, float z, int lifetime)
{
	gaussians.emplace_back(x, y, z, lifetime);
}

float GaussianMap::calculate(float x, float y, float z)
{
	// Using the multivariate normal distribution, calculate the results of the
	// specified 3D point for each gaussian point in the map, using the lifetime
	// of each to specify the width of their "bells".
	float result = 0.0f;
	const float pi = 3.141592654f;
	for (const auto &gaussian : gaussians)
	{
		const float coeff = 1.0f / (gaussian.lifetime * std::sqrt(2.0f * pi));
		float i = coeff * std::exp(-std::pow(x - gaussian.x, 2) / (2.0f * std::pow(gaussian.lifetime, 2)));
		float j = coeff * std::exp(-std::pow(y - gaussian.y, 2) / (2.0f * std::pow(gaussian.lifetime, 2)));
		float k = coeff * std::exp(-std::pow(z - gaussian.z, 2) / (2.0f * std::pow(gaussian.lifetime, 2)));

		float product = i * j * k * 1000.0f;
		result += product;
	}
	return result;
}

void GaussianMap::decay(float rate)
{
	for (size_t i = 0; i < gaussians.size(); i++)
	{
		gaussians[i].lifetime -= rate;
		if (gaussians[i].lifetime <= 0)
		{
			gaussians.erase(gaussians.begin() + i);
		}
	}
}