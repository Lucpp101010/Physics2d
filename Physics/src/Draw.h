#pragma once

#include "Simulation.h"
#include "physics/Math.h"
#include "visu/Vao.h"
#include "visu/Shader.h"

class Scene
{
public:
	
	Simulation simulation;
	Vec corner1, corner2;
	double screenWidth, screenHeight, aspectRatio;

	Vao vaoCircle;
	Program shader;

	void init();

	Vec fromScreenPoint(const Vec& p) const;
	Vec toScreenPoint(const Vec& p) const;
	Vec toGlPoint(const Vec& p) const;

	void draw();
	void update(double dt);
};