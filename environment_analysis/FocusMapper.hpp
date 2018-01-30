#ifndef _GAUSSIANMAP_H_
#define _GAUSSIANMAP_H_

#include <unordered_map>
#include <vector>

struct FocusPoint
{
	float x, y, z;
	float radius;

	FocusPoint(float x, float y, float z, float radius) :
		x(x), y(y), z(z), radius(radius) {}
};

class FocusMapper
{
public:
	FocusMapper();

	void add(float x, float y, float z, float radius = 10.0f);
	float calculate(float x, float y, float z);
	void decay(float rate = 1.0f);
private:
	std::vector<FocusPoint> points;
};

#endif