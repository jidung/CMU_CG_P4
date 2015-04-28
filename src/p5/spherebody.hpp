#ifndef _462_PHYSICS_SPHEREBODY_HPP_
#define _462_PHYSICS_SPHEREBODY_HPP_

#include "scene/sphere.hpp"
#include "p5/body.hpp"

namespace _462 {

class Sphere;

class SphereBody : public Body
{
public:
    Sphere* sphere;
    real_t radius;
    real_t mass;
    Vector3 force;
    Vector3 torque;

    Vector3 angular_accel; //jd

    SphereBody( Sphere* geom );
    virtual ~SphereBody() { }
    Vector3 acceleration( real_t dt, Vector3 velocity, real_t motion_damping );
    Vector3 angular_acceleration( real_t dt, Vector3 velocity, real_t motion_damping );
    virtual Vector3 step_position( real_t dt, real_t motion_damping );
    virtual Vector3 step_orientation( real_t dt, real_t motion_damping );
    virtual void apply_force( const Vector3& f, const Vector3& offset );
    void apply_gravity( const Vector3& gravity );
};

}

#endif

