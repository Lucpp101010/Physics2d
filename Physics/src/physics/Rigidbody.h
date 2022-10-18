#pragma once
#include <vector>

#include "Math.h"
#include "Shapes.h"

class RigidBody
{
public:

	Vec pos;
	int shape;
	double mass;
	Vec v, a;
	double angle, omega;

	// stores the time that remains to be simulated during simulation frame
	double time_left;

	bool active = true;
	bool fixed = false;

	double getInertia() const;

	bool isPointInside(const Vec& p) const;

	std::vector<Vec> getVertices() const;

	Vec getPenetrationVec(const Vec& p) const;
};

void elasticCollision(RigidBody& b1, RigidBody& b2, Vec p, Vec n, double e, double mu);