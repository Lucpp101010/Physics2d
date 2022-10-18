#pragma once

#include <vector>
#include <map>

#include "physics/Shapes.h"
#include "physics/Rigidbody.h"

#include "visu/Shader.h"
#include "visu/Vao.h"

class Simulation
{
public:
	// actual physics simulation
	std::vector<RigidBody> rigidBodies;
	int shapeCnt = 0;

	// related to drawing
	double screenWidth, screenHeight;
	Vec corner1, corner2;
	Program shader;

	void simulate(double dt);

	void draw();

	Vec fromScreenPoint(const Vec& p) const;
	Vec toScreenPoint(const Vec& p) const;
	Vec toGlPoint(const Vec& p) const;

	std::vector<RigidBody*> atPoint(const Vec& v);
};