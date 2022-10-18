#include "Rigidbody.h"

double RigidBody::getInertia() const
{
	return shapes[shape].inertia * mass;
}

bool RigidBody::isPointInside(const Vec& p) const
{
	Vec p2 = { 130.77345, 432.124124 };
	std::vector<Vec> vert = getVertices();
	int cnt = 0;
	for (int i = 0; i < (int)vert.size(); ++i)
	{
		cnt += segmentIntersect(p, p2, vert[i], vert[(i + 1) % (int)vert.size()]);
	}
	return cnt % 2 == 1;
}

std::vector<Vec> RigidBody::getVertices() const
{
	std::vector<Vec> res;
	const Shape& s = shapes[shape];
	double si = sin(angle);
	double co = cos(angle);
	for (const Vec& v : s.vertices)
	{
		res.push_back(pos + v * co + v.perp() * si);
	}
	return res;
}

Vec RigidBody::getPenetrationVec(const Vec& p) const
{
	Vec ans = { 10000, 10000 };
	std::vector<Vec> vertices = getVertices();
	for (int i = 0; i < (int)vertices.size(); ++i)
	{
		Vec n = (vertices[(i + 1) % (int)vertices.size()] - vertices[i]).perp().normalize();
		Vec v = n * n.dot(p - vertices[i]);
		if (segmentIntersect(vertices[i], vertices[(i + 1) % (int)vertices.size()], p, p - v * 2))
		{
			if (v.lenSq() < ans.lenSq()) ans = v;
		}
	}
	return ans;
}

#include <iostream>

void elasticCollision(RigidBody& b1, RigidBody& b2, Vec p, Vec n, double e, double mu)
{
	if ((b1.fixed||!b1.active) && (b2.fixed||!b2.active)) return;

	Vec r1 = b1.pos - p;
	Vec r2 = b2.pos - p;
	Vec vel1 = b1.v - r1.perp() * b1.omega;
	Vec vel2 = b2.v - r2.perp() * b2.omega;
	// check that bodies are not already moving away
	if ((vel1 - vel2).dot(n) < -epsilon)
	{
		double impulse;
		if(b1.fixed || !b1.active) impulse = -(1.0 + e) * (vel1 - vel2).dot(n) / (1.0 / b2.mass + r2.cross(n) * r2.cross(n) / b2.getInertia());
		else if(b2.fixed || !b2.active) impulse = -(1.0 + e) * (vel1 - vel2).dot(n) / (1.0 / b1.mass + r1.cross(n) * r1.cross(n) / b1.getInertia());
		else impulse = -(1.0 + e) * (vel1 - vel2).dot(n) / (1.0 / b1.mass + 1.0 / b2.mass + r1.cross(n) * r1.cross(n) / b1.getInertia() + r2.cross(n) * r2.cross(n) / b2.getInertia());
		//impulse += 1e-16;
		double tangent = (vel1 - vel2).dot(n.perp());
		double sign = (tangent > 0 ? -1.0 : 1.0);
		Vec friction = n.perp() * std::min(mu * impulse, std::abs(tangent / (1.0 / b1.mass + 1.0 / b2.mass + r1.cross(n.perp()) / b1.getInertia() + r2.cross(n.perp())))) * sign;
		//std::cout << friction.x << " " << friction.y << " " << tangent << " " << mu*impulse << std::endl;
		if (!(b1.fixed || !b1.active))
		{
			b1.v += (n * impulse + friction) / b1.mass;
			b1.omega += r1.cross(n * impulse + friction) / b1.getInertia();
		}
		if (!(b2.fixed || !b2.active))
		{
			b2.v -= (n * impulse + friction) / b2.mass;
			b2.omega -= r2.cross(n * impulse + friction) / b2.getInertia();
		}

		Vec v1 = b1.v - r1.perp() * b1.omega;
		Vec v2 = b2.v - r2.perp() * b2.omega;
		//std::cout << b1.omega << " " << b1.v.y << std::endl;
		//std::cout << (vel1 - vel2).dot(n) << " " << (v1 - v2).dot(n) << std::endl;
		//std::cout << v1.dot(n) << " " << b1.v.dot(n) + r1.cross(n) * b1.omega << std::endl;
	}
}
