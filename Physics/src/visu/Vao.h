#pragma once

#include <glad/glad.h>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

#include "..\physics\Math.h"

class Vertex
{
public:
	float x, y, tx, ty;
};

class Vao
{
private:
	GLuint id;
	GLuint buf;

public:
	int numVertices;

	Vao() = default;
	Vao(const std::vector<Vertex> &v);
	Vao(const std::vector<Vec>& v);

	GLuint getId();

	static Vao createRect();
	static Vao createCircle(int points);

	void remove();
};