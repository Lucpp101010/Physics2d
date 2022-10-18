#include "Rigidbody.h"

struct CollisionInfo 
{
	bool collision;
	double depth;
	std::vector<Vec> points;
	Vec normal;
};

CollisionInfo calcCollisionInfo(RigidBody& body1, RigidBody& body2);