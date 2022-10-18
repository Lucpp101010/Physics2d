#pragma once
#include "Collision.h"
#include <iostream>

CollisionInfo calcCollisionInfo(RigidBody& body1, RigidBody& body2)
{
	CollisionInfo info{ false };
	// check if colliding
	std::vector<Vec> vertices = body1.getVertices();
	int sz = vertices.size();
	for (int i = 0; i < sz; ++i)
	{
		Vec v = vertices[i];
		//Vec next = vertices[(i + 1) % sz];
		//Vec mid = (v + next) / 2;
		if (body2.isPointInside(v) /*|| body2.isPointInside(mid)*/) info.collision = true;
	}
	if(!info.collision) return info;

	// find normal and point
	std::vector<Vec> vertices2 = body2.getVertices();
	int sz2 = vertices2.size();
	double minTranslate = 1e18;
	for (int i = 0; i < sz2; ++i)
	{
		double maxTranslate = 0;
		std::vector<Vec> points;
		Vec normal = (vertices2[(i + 1) % sz2] - vertices2[i]).perp().normalize();
		for (int j = 0; j < sz; ++j)
		{
			for (int k = 0; k < sz2; ++k)
			{
				if (segmentIntersect(vertices2[k], vertices2[(k + 1) % sz2], vertices[j], vertices[j] + normal * 1e7))
				{
					std::optional<Vec> p = lineIntersect(Line(vertices2[k], vertices2[(k + 1) % sz2]), Line(vertices[j], vertices[j] + normal * 10.0));
					if (p.has_value())
					{
						double translate = (vertices[j] - *p).len();
						if (maxTranslate < translate)
						{
							maxTranslate = translate;
						}
						points.push_back(vertices[j]);
					}
				}
			}
		}
		if (maxTranslate < minTranslate)
		{
			minTranslate = maxTranslate;
			info.normal = normal;
			info.points = points;
			info.depth = maxTranslate;
		}
	}
	//// if bodies are moving apart, then collision response is not needed (seems to have no effect...)
	//info.collision = false;
	//for(Vec p : vertices)
	//{
	//	if (!body2.isPointInside(p)) continue;
	//	Vec r1 = body1.pos - p;
	//	Vec r2 = body2.pos - p;
	//	Vec vel1 = body1.v - r1.perp() * body1.omega;
	//	Vec vel2 = body2.v - r2.perp() * body2.omega;
	//	//if(info.normal.dot(vel1 - vel2) < -epsilon) std::cout << "$ " << p.x << " " << p.y << " " << info.normal.dot(vel1 - vel2) << std::endl;
	//	if (info.normal.dot(vel1 - vel2) < -epsilon) info.collision = true;
	//}
	return info;
}
