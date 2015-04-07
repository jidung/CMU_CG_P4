#include "p5/collisions.hpp"

namespace _462 {

bool collides( SphereBody& body1, SphereBody& body2, real_t collision_damping )
{
    Vector3 v1_, v2_, d, v2__, u1, u2;

    Vector3 p2minusp1 = body1.position - body2.position;

    // TODO detect collision. If there is one, update velocity
    if ( squared_length (p2minusp1) 
            < (body1.radius + body2.radius)*(body1.radius + body2.radius) )
    {
        v1_ = body1.velocity - body2.velocity;
        v2_ = Vector3::Zero();
        
        //d = p2minusp1 / distance (p2minusp1);
        d = normalize (p2minusp1);  // should be the same thing

        v2__ = 2.0*d*body1.mass*(dot(v1_, d)) / (body1.mass + body2.mass);

        u2 = body2.velocity + v2__;

        u1 = (body1.mass*body1.velocity + body2.mass*body2.velocity - body2.mass*u2) / body1.mass;

        // if the relative velocity of one object to the other is positive
        // can't it be just squared_length(u1-u2) != 0.0 ?
        if ( squared_length(u1-u2) > 0.0 || squared_length(u2-u1) > 0.0 )
        {
            body1.velocity = u1;
            body2.velocity = u2;

            std::cout << "Sphere-Sphere collision detected" << std::endl;
            return true; //collides
        }
    }

    return false;
}

bool collides( SphereBody& body1, TriangleBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
    Vector3 v1 = body2.vertices[0];
    Vector3 v2 = body2.vertices[1];
    Vector3 v3 = body2.vertices[2];

    Vector3 n = normalize ( cross (v2 - v1, v3 - v1) );  // triangle normal

    Vector3 p1 = body1.position;
    Vector3 p2 = body2.position;
    Vector3 a = p1 - p2;
    real_t  d = dot(a, n);   // distacne (projection)
    
    // point on the plane defined by the triangle
    Vector3 p = p1 - d * n; 

    Vector3 vToP;

    Vector3 p_;

    real_t u, v, w;     // for barycentric coordinates

    Vector3 edge = v2 - v1;
    vToP = p - v1;
    w = dot (n, cross(edge, vToP));
    
    edge = v3 - v2;
    vToP = p - v2;
    u = dot (n, cross(edge, vToP));

    edge = v1 - v3;
    vToP = p - v3;
    v = dot (n, cross(edge, vToP));

    if ( w >= 0 && u >= 0 && v >= 0) {
        if ( fabs(d) < body1.radius )
        {  
            std::cout << "Sphere-triangle collision detected" << std::endl;
            return true;
        }
    }
    else
    {
        p_ = dot(p-v1, v2-v1) * (v2-v1) / squared_length(v2-v1) + v1;
        if ( squared_distance(p1, p_) < body1.radius * body1.radius)
            return true;
        p_ = dot(p-v1, v3-v1) * (v3-v1) / squared_length(v3-v1) + v1;
        if ( squared_distance(p1, p_) < body1.radius * body1.radius)
            return true;
        p_ = dot(p-v3, v2-v3) * (v2-v3) / squared_length(v2-v3) + v3;
        if ( squared_distance(p1, p_) < body1.radius * body1.radius)
            return true;
    }

    // collides
    
    /*
    real_t squared_length_normal = squared_length (n); 
    v = v / squared_length_normal;
    u = u / squared_length_normal;
    w = 1 - v - u;
    */


    return false;
}

bool collides( SphereBody& body1, PlaneBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
    Vector3 p1 = body1.position;
    Vector3 p2 = body2.position;
    Vector3 a = p1 - p2;
    real_t d = dot(a, body2.normal);

    //std::cout << p2 << std::endl;

    if ( fabs(d) < body1.radius ) { // collision
        Vector3 u;
        u = body1.velocity - 2.0 * dot (body1.velocity, body2.normal) * body2.normal;
        body1.velocity = u;
        std::cout << "Sphere-Plane collision detected" << std::endl;
        return true;
    }
	return false;
}

bool collides( SphereBody& body1, ModelBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
    const MeshTriangle* triangles = body2.model->mesh->get_triangles();
    const MeshVertex* vertices = body2.model->mesh->get_vertices(); 

    bool hit = false;
    real_t u, v, w; // for barycentric coord calculation
        
    Vector3 p1 = body1.position;
    //Vector3 p1 = Vector3::Zero();

    Vector3 p_;

    for (unsigned int i = 0; i < body2.model->mesh->num_triangles(); ++i) {
        
        Vector3 v1 = vertices[triangles[i].vertices[0]].position + body2.position;
        Vector3 v2 = vertices[triangles[i].vertices[1]].position + body2.position;
        Vector3 v3 = vertices[triangles[i].vertices[2]].position + body2.position;
        
        /*
        Vector3 v1 = vertices[triangles[i].vertices[0]].position;
        Vector3 v2 = vertices[triangles[i].vertices[1]].position;
        Vector3 v3 = vertices[triangles[i].vertices[2]].position;
        */
//        Vector3 n = normalize ( cross (v3 - v1, v2 - v1) );  // triangle normal
        Vector3 n = normalize ( cross (v2 - v1, v3 - v1) );  // triangle normal

        //Vector3 p2 = (v1 + v2 + v3) / 3.0;    // any point on the trianlge. this time its centroid
        Vector3 p2 = v1;
        Vector3 a = p1 - p2;
        real_t  d = dot(a, n);   // distacne (projection)

        // point on the plane defined by the triangle
        Vector3 p = p1 - d * n; 

        Vector3 vToP;

        Vector3 edge = v2 - v1;
        vToP = p - v1;
        w = dot (n, cross(edge, vToP));

        edge = v3 - v2;
        vToP = p - v2;
        u = dot (n, cross(edge, vToP));

        edge = v1 - v3;
        vToP = p - v3;
        v = dot (n, cross(edge, vToP));

        if ( w >= 0 && u >= 0 && v >= 0) {
            if ( fabs(d) < body1.radius )
            {
                std::cout << "Sphere-Model collision detected1" << std::endl;
                return true;
            }
        }
        else
        {
            /*
            p_ = dot(p-v1, v2-v1) * (v2-v1) / squared_length(v2-v1) + v1;
            if ( squared_distance(p1, p_) < body1.radius * body1.radius) {
                std::cout << "Sphere-Model collision detected2" << std::endl;
                return true;
            }
            p_ = dot(p-v1, v3-v1) * (v3-v1) / squared_length(v3-v1) + v1;
            if ( squared_distance(p1, p_) < body1.radius * body1.radius ) {
                std::cout << "Sphere-Model collision detected3" << std::endl;
                return true;
            }
            p_ = dot(p-v3, v2-v3) * (v2-v3) / squared_length(v2-v3) + v3;
            if ( squared_distance(p1, p_) < body1.radius * body1.radius) {
                std::cout << "Sphere-Model collision detected4" << std::endl;
                return true;
            }
            */
        }
    }

    std::cout << "Sphere-Model collision NOT detected" << std::endl;
    return false;
}

}
