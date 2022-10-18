#include "Math.h"
#include <math.h>

Vec::Vec(double x, double y) : x(x), y(y)
{
}

double Vec::len() const
{
	return sqrt(this->lenSq());
}

double Vec::lenSq() const
{
	return x * x + y * y;
}

double Vec::dist(const Vec& v) const
{
	return sqrt(this->distSq(v));
}

double Vec::distSq(const Vec& v) const
{
	return (*this - v).lenSq();
}

Vec Vec::normalize() const
{
	return *this / len();
}

Vec Vec::perp() const
{
	return Vec(-y, x);
}

double Vec::dot(const Vec& v) const
{
	return x * v.x + y * v.y;
}

double Vec::cross(const Vec& v) const
{
	return y * v.x - x * v.y;
}

Vec Vec::operator+(const Vec& v) const
{
	return { x + v.x, y + v.y };
}

Vec Vec::operator-(const Vec& v) const
{
	return { x - v.x, y - v.y };
}

Vec Vec::operator*(double scale) const
{
	return { scale * x, scale * y };
}

Vec Vec::operator/(double scale) const
{
	return { x / scale, y / scale };
}

void Vec::operator+=(const Vec& v)
{
	x += v.x; y += v.y;
}

void Vec::operator-=(const Vec& v)
{
	x -= v.x; y -= v.y;
}

void Vec::operator*=(double scale)
{
	x *= scale; y *= scale;
}

void Vec::operator/=(double scale)
{
	x /= scale; y /= scale;
}

bool segmentIntersect(const Vec& p1, const Vec& p2, const Vec& q1, const Vec& q2)
{
	double c1 = (p2 - p1).cross(q1 - p1);
	double c2 = (p2 - p1).cross(q2 - p1);
	if ((c1 > 0 && c2 > 0) || (c1 < 0 && c2 < 0)) return false;
	c1 = (q2 - q1).cross(p1 - q1);
	c2 = (q2 - q1).cross(p2 - q1);
	if ((c1 > 0 && c2 > 0) || (c1 < 0 && c2 < 0)) return false;
	return true;
}

std::optional<Vec> lineIntersect(const Line& g, const Line& h)
{
	double det = g.a * h.b - g.b * h.a;
	if (det == 0.0) return {};
	double x = (g.b * h.c - g.c * h.b) / det;
	double y = (g.c * h.a - g.a * h.c) / det;
	return Vec(x, y);
}

Line::Line(double a, double b, double c) : a(a), b(b), c(c) {}

Line::Line(const Vec& p, const Vec& q)
{
	a = q.y - p.y;
	b = p.x - q.x;
	c = -(a * p.x + b * p.y);
}
