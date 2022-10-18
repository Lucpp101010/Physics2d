#include "Shapes.h"

bool Circle::isPointInside(const Vec& p) const
{
	return center.distSq(p) <= radius * radius;
}

static int shapeCnt = 0;
std::map<int, Shape> shapes;

void addShape(Shape shape)
{
	shape.id = shapeCnt;
	shapes[shapeCnt++] = shape;
}

void removeShape(int id)
{
	shapes[id].vao.remove();
	shapes.erase(id);
}

Shape::Shape(std::vector<Vec> vertices, double inertia, Color color)
{
	this->vertices = vertices;
	this->inertia = inertia;
	this->vao = Vao(vertices);
	this->color = color;
}
