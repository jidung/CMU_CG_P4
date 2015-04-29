#ifndef _462_PHYSICS_PHYSICS_HPP_
#define _462_PHYSICS_PHYSICS_HPP_

#include "math/math.hpp"
#include "math/quaternion.hpp"
#include "math/vector.hpp"
#include "p5/body.hpp"
#include "p5/spherebody.hpp"
#include "p5/trianglebody.hpp"
#include "p5/planebody.hpp"
#include "p5/modelbody.hpp"
#include "p5/spring.hpp"
#include "p5/collisions.hpp"

#include <vector>

namespace _462 {

class Physics
{
public:
    Vector3 gravity;
	real_t collision_damping;
	real_t motion_damping;  // m.ji
    real_t rotation_damping; // m.ji

    Physics();
    ~Physics();

    void step( real_t dt );
    void add_sphere( SphereBody* s );
    size_t num_spheres() const;
    void add_plane( PlaneBody* p );
    size_t num_planes() const;
    void add_triangle( TriangleBody* t );
    size_t num_triangles() const;
    void add_model( ModelBody* m );
    size_t num_models() const;
    void add_spring( Spring* s );
    size_t num_springs() const;

    void reset();

private:
    typedef std::vector< Spring* > SpringList;
    typedef std::vector< SphereBody* > SphereList;
    typedef std::vector< PlaneBody* > PlaneList;
    typedef std::vector< TriangleBody* > TriangleList;
    typedef std::vector< ModelBody* > ModelList;

    SpringList springs;
    SphereList spheres;
    PlaneList planes;
    TriangleList triangles;
    ModelList models;
    
    Vector3 rk4( Vector3(Physics::*f)(Vector3, Vector3, real_t), Vector3 x, Vector3 current_x, real_t dt )   //m.ji
    {
        Vector3 k1 = dt * (*this.*f)(x, current_x, dt),
                k2 = dt * (*this.*f)(x + k1 * 0.5, current_x, dt * 0.5),
                k3 = dt * (*this.*f)(x + k2 * 0.5, current_x, dt * 0.5),
                k4 = dt * (*this.*f)(x + k3, current_x, dt);

        return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
    }
    
    Vector3 evaluate(Vector3 x, Vector3 current_x, real_t dt)
    {
        return x * dt + current_x; 
    }

};

}

#endif

