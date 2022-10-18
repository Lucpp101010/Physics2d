#pragma once
#include <optional>

const double epsilon = 1e-6;

class Vec
{
public:
	Vec() = default;
	Vec(double x, double y);

	double x, y;

	double len() const;
	double lenSq() const;
	double dist(const Vec& v) const;
	double distSq(const Vec& v) const;

	Vec normalize() const;
	Vec perp() const;

	double dot(const Vec& v) const;
	double cross(const Vec& v) const;

	Vec operator + (const Vec& v) const;
	Vec operator - (const Vec& v) const;
	Vec operator * (double scale) const;
	Vec operator / (double scale) const;

	void operator += (const Vec& v);
	void operator -= (const Vec& v);
	void operator *= (double scale);
	void operator /= (double scale);
};


// check if line segments p1 - p2 and q1 - q2 intersect
bool segmentIntersect(const Vec& p1, const Vec& p2, const Vec& q1, const Vec& q2);

struct Line
{
	Line(double a, double b, double c);
	Line(const Vec& p, const Vec& q);
	// a*x + b*y + c = 0
	double a, b, c;
};

std::optional<Vec> lineIntersect(const Line& g, const Line& h);
