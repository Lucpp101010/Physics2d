#pragma once
#include <vector>
#include <map>
#include "Math.h"
#include "../visu/Color.h"
#include "..\visu\Vao.h"

class Circle
{
public:

	Vec center;
	double radius;
	Vec v, a;
	bool active;
	
	bool isPointInside(const Vec& p) const;
};

class Shape
{
public:

	std::vector<Vec> vertices;
	double inertia;
	Vao vao;
	Color color;
	int id;

	Shape() = default;

	Shape(std::vector<Vec> vertices, double inertia, Color color);
};

extern std::map<int, Shape> shapes;

void addShape(Shape shape);
void removeShape(int id);
