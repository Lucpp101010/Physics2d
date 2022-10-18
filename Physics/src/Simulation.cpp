#include "Simulation.h"
#include "physics/Collision.h"
#include <iostream>
#include <random>

//Vec elasticCollision(double v1, double m1, double v2, double m2)
//{
//    Vec ans;
//    ans.x = ((m1 - m2) * v1 + 2.0 * m2 * v2) / (m1 + m2);
//    ans.y = ((m2 - m1) * v2 + 2.0 * m1 * v1) / (m1 + m2);
//    return ans;
//}

std::mt19937 mt;

void Simulation::simulate(double dt)
{
    auto integrate = [&](RigidBody& b, double t)
    {
        if (b.fixed || !b.active) return;
        b.pos += b.v * t;
        b.angle += b.omega * t;
        b.time_left -= t;
    };

    for (RigidBody& body : rigidBodies)
    {
        body.time_left = dt;
        if (!body.active || body.fixed) continue;
        body.a = { 0, -9.81 };
        body.v += body.a * dt;
        //body.pos += body.v * dt;
        //body.angle += body.omega * dt;

        //std::vector<Vec> vert = body.getVertices();
        //for (Vec v : vert)
        //{
        //    Vec r = body.pos - v;
        //    Vec vel = body.v - r.perp() * body.omega;
        //    if (v.y <= 0 && vel.y < 0)
        //    {
        //        Vec n = { 0, 1 };
        //        double impulse = -2.0 * vel.y / (1.0 / body.mass + r.cross(n) * r.cross(n) / body.getInertia());
        //        body.v.y += impulse / body.mass;
        //        body.omega += impulse * r.cross(n) / body.getInertia();

        //        body.pos.y += dt * (impulse / body.mass);
        //        body.angle += dt * (impulse * r.cross(n) / body.getInertia());
        //        Vec vel2 = body.v - r.perp() * body.omega;
        //        //std::cout << body.v.y << std::endl;
        //        //std::cout << vel.dot(n) << " " << vel2.dot(n) << std::endl;
        //    }
        //    if (v.y >= 100.0) body.v.y = -fabs(body.v.y);
        //    if (v.x <= 0)     body.v.x =  fabs(body.v.x);
        //    if (v.x >= 100.0) body.v.x = -fabs(body.v.x);
        //} 
    }


    for(int exec = 0; exec < 30; ++exec)
    for (int i = 0; i < rigidBodies.size(); ++i)
    {
        for (int j = i + 1; j < rigidBodies.size(); ++j)
        {
            RigidBody* a = &rigidBodies[i];
            RigidBody* b = &rigidBodies[j];
            if (a->fixed && b->fixed) continue;

            double time_a = a->time_left;
            double time_b = b->time_left;
            integrate(*a, time_a);
            integrate(*b, time_b);
            bool colliding = calcCollisionInfo(*a, *b).collision || calcCollisionInfo(*b, *a).collision;
            integrate(*a, -time_a);
            integrate(*b, -time_b);
            if (colliding)
            {
                //if (exec > 0) std::cout << exec << std::endl;
                if (time_a < time_b) integrate(*b, time_b - time_a);
                if (time_b < time_a) integrate(*a, time_a - time_b);
                time_a = a->time_left; time_b = b->time_left;
                CollisionInfo info1, info2;
                for (double t = std::min(time_a, time_b) / 2; t > epsilon; t /= 2)
                {
                    //std::cout << t << std::endl;
                    integrate(*a, t);
                    integrate(*b, t);
                    CollisionInfo i1 = calcCollisionInfo(*a, *b);
                    CollisionInfo i2 = calcCollisionInfo(*b, *a);
                    if (i1.collision || i2.collision)
                    {
                        integrate(*a, -t);
                        integrate(*b, -t);
                        info1 = i1;
                        info2 = i2;
                    }
                }
                //std::cout << info1.collision << " # " << info2.collision << std::endl;
                for (int it = 0; it < 2; ++it)
                {
                    if (info1.collision)
                    {
                        std::shuffle(info1.points.begin(), info1.points.end(), mt);
                        for (Vec v : info1.points)
                        {
                            elasticCollision(*a, *b, v, info1.normal, 0.5, 1);
                        }
                        //Vec mid(0, 0);
                        //for (Vec v : info1.points) mid += v;
                        ////std::cout << info1.points.size() << std::endl;
                        //mid /= info1.points.size();
                        ////std::cout << "# " << mid.x << " " << mid.y << std::endl;
                        ////std::cout << "# " << info1.normal.x << " " << info1.normal.y << std::endl;
                        //elasticCollision(*a, *b, mid, info1.normal, 0, 1);
                        ////std::cout << a->v.y << " " << a->omega << " " << info1.points.size() << std::endl;
                    }
                    std::swap(a, b);
                    std::swap(info1, info2);
                }
            }
            //for (int it = 0; it < 2; ++it)
            //{
            //    //for (Vec vert : a->getVertices())
            //    //{
            //    //    if (b->isPointInside(vert))
            //    //    {
            //    //        Vec n = b->getPenetrationVec(vert).normalize() * -1;
            //    //        //std::cout << n.x << " " << n.y << std::endl;
            //    //        elasticCollision(*a, *b, vert, n, 0);
            //    //    }
            //    //}
            //    CollisionInfo info = calcCollisionInfo(*a, *b);
            //    if (info.collision)
            //    {
            //        Vec mid(0, 0);
            //        for (Vec v : info.points) mid += v;
            //        mid /= info.points.size();
            //        if (b->fixed)
            //        {
            //            a->pos += info.normal * info.depth;
            //            mid += info.normal * info.depth;
            //        }
            //        else if (a->fixed)
            //        {
            //            b->pos -= info.normal * info.depth;
            //            mid -= info.normal * info.depth;
            //        }
            //        else
            //        {
            //            a->pos += info.normal * info.depth / 2;
            //            b->pos -= info.normal * info.depth / 2;
            //        }
            //        elasticCollision(*a, *b, mid, info.normal, 0);
            //    }
            //    std::swap(a, b);
            //}

            //if (a.center.distSq(b.center) > (a.radius + b.radius) * (a.radius + b.radius)) continue;

            //Vec normal = (b.center - a.center).normalize();
            //Vec tangent = Vec(-normal.y, normal.x);
            //double va = normal.dot(a.v);
            //double vb = normal.dot(b.v);

            //if (va < 0 && vb > 0) continue;

            //Vec after = elasticCollision(va, a.radius*a.radius, vb, b.radius*b.radius);
            //a.v = tangent * tangent.dot(a.v) + normal * after.x;
            //b.v = tangent * tangent.dot(b.v) + normal * after.y;

            ////double force = std::max(0.0, (a.radius + b.radius) - a.center.dist(b.center));
            ////a.v += (a.center - b.center).normalize() * force;
            ////b.v += (b.center - a.center).normalize() * force;
        }
    }

    for (RigidBody& body : rigidBodies)
    {
        if (!body.active || body.fixed) continue;
        //body.a = { 0, -9.81 };
        //body.v += body.a * dt;
        //if (body.v.len() > epsilon) body.pos += body.v * dt;
        //if(abs(body.omega) > epsilon) body.angle += body.omega * dt;
        //std::cout << body.time_left << std::endl;
        integrate(body, body.time_left);
        //std::cout << body.shape << " " << body.v.y << std::endl;
    }
}

std::vector<RigidBody*> Simulation::atPoint(const Vec& v)
{
    std::vector<RigidBody*> res;
    for (int i = 0; i < rigidBodies.size(); ++i)
    {
        if (rigidBodies[i].isPointInside(v)) res.push_back(&rigidBodies[i]);
    }
    return res;
}

Vec Simulation::fromScreenPoint(const Vec& p) const
{
    Vec v;
    v.x = corner1.x + (p.x / screenWidth) * (corner2.x - corner1.x);
    v.y = corner1.y + ((screenHeight - p.y) / screenHeight) * (corner2.y - corner1.y);
    return v;
}

Vec Simulation::toScreenPoint(const Vec& p) const
{
    Vec v;
    v.x = (p.x / (corner2.x - corner1.x)) * screenWidth;
    v.y = ((screenHeight - p.y) / (corner2.y - corner1.y)) * screenHeight;
    return v;
}

Vec Simulation::toGlPoint(const Vec& p) const
{
    Vec v;
    v.x = 2.0 * (p.x - corner1.x) / (corner2.x - corner1.x) - 1;
    v.y = 2.0 * (p.y - corner1.y) / (corner2.y - corner1.y) - 1;
    return v;
}

void Simulation::draw()
{
    shader.use();
    //glBindVertexArray(vaoCircle.getId());
    double scaleX = 2.0 / (corner2.x - corner1.x);
    double scaleY = 2.0 / (corner2.y - corner1.y);
    for (const RigidBody& body : rigidBodies)
    {
        glBindVertexArray(shapes[body.shape].vao.getId());
        Vec p = toGlPoint(body.pos);
        shader.setUniformVec2("trans", p.x, p.y);
        shader.setUniformFloat("angle", body.angle);
        shader.setUniformVec2("scale", scaleX, scaleY);
        Color col = shapes[body.shape].color;
        shader.setUniformVec3("color", col.r, col.g, col.b);
        glDrawArrays(GL_TRIANGLE_FAN, 0, shapes[body.shape].vao.numVertices);
    }
}
