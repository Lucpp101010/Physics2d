#include <glad/glad.h>
#include "Draw.h"
#include <iostream>

void Scene::init()
{
	shader = shader.load("src/shader/shader");
	vaoCircle = Vao::createCircle(64);
}

Vec Scene::fromScreenPoint(const Vec& p) const
{
	Vec v;
	v.x = corner1.x + (p.x / screenWidth) * (corner2.x - corner1.x);
	v.y = corner1.y + ((screenHeight - p.y) / screenHeight) * (corner2.y - corner1.y);
	return v;
}

Vec Scene::toScreenPoint(const Vec& p) const
{
	Vec v;
	v.x = (p.x / (corner2.x - corner1.x)) * screenWidth;
	v.y = ((screenHeight - p.y) / (corner2.y - corner1.y)) * screenHeight;
	return v;
}

Vec Scene::toGlPoint(const Vec& p) const
{
	Vec v;
	v.x = 2.0 * (p.x - corner1.x) / (corner2.x - corner1.x) - 1;
	v.y = 2.0 * (p.y - corner1.y) / (corner2.y - corner1.y) - 1;
	return v;
}

void Scene::draw()
{
	//shader.use();
	//glBindVertexArray(vaoCircle.getId());
	//double scaleX = 2.0 / (corner2.x - corner1.x);
	//double scaleY = 2.0 / (corner2.y - corner1.y);
	//for (Circle& c : simulation.circles)
	//{
	//	Vec v = toGlPoint(c.center);
	//	shader.setUniformVec2("trans", v.x, v.y);
	//	shader.setUniformVec2("scale", c.radius*scaleX, c.radius*scaleY);
	//	glDrawArrays(GL_TRIANGLE_FAN, 0, 64);
	//}
}

void Scene::update(double dt)
{
	simulation.simulate(dt);
}
