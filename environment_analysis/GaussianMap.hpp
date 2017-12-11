#ifndef _GAUSSIANMAP_H_
#define _GAUSSIANMAP_H_

#include <unordered_map>
#include <vector>

struct Gaussian
{
	float x, y, z;
	float lifetime;

	Gaussian(float x, float y, float z, float lifetime) :
		x(x), y(y), z(z), lifetime(lifetime) {}
};

class GaussianMap
{
public:
	GaussianMap();

	void add(float x, float y, float z, int lifetime = 10);
	float calculate(float x, float y, float z);
	void decay(float rate = 1.0f);
private:
	std::vector<Gaussian> gaussians;
};

#endif