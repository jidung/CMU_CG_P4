#include "p5/spherebody.hpp"
#include "math/vector.hpp"
#include "math/matrix.hpp"
#include "scene/sphere.hpp"
#include <iostream>
#include <exception>
#include <algorithm>

namespace _462 {

SphereBody::SphereBody( Sphere* geom )
{
    sphere = geom;
    position = sphere->position;
    radius = sphere->radius;
    orientation = sphere->orientation;
    mass = 0.0;
    velocity = Vector3::Zero();
    angular_velocity = Vector3::Zero();
    force = Vector3::Zero();
    torque = Vector3::Zero();
}

Vector3 SphereBody::step_position( real_t dt, real_t motion_damping )
{
    // Note: This function is here as a hint for an approach to take towards
    // programming RK4, you should add more functions to help you or change the
    // scheme
    // TODO return the delta in position dt in the future

    //return Vector3::Zero();

    Vector3 deltaPos;
    deltaPos = position + velocity * dt;
    velocity = (velocity + (force / mass) * dt) * motion_damping;

    return deltaPos;
}

Vector3 SphereBody::step_orientation( real_t dt, real_t motion_damping )
{
    // Note: This function is here as a hint for an approach to take towards
    // programming RK4, you should add more functions to help you or change the
    // scheme
    // TODO return the delta in orientation dt in the future
    // vec.x = rotation along x axis
    // vec.y = rotation along y axis
    // vec.z = rotation along z axis

    Quaternion rotate_quaternion (0.0, angular_velocity.x, angular_velocity.y,
            angular_velocity.z);
    Quaternion spin;

    spin = 0.5 * rotate_quaternion * orientation * dt; 
   
    spin.w = spin.w + orientation.w;
    spin.x = spin.x + orientation.x;
    spin.y = spin.y + orientation.y;
    spin.z = spin.z + orientation.z;

    orientation = normalize(spin);
    //orientation = normalize ( Quaternion ( Vector3::UnitX(), 0.1 ) * orientation );

    angular_velocity = (angular_velocity + (torque / mass) * dt) * motion_damping;

    return Vector3::Zero();
}

void SphereBody::apply_force( const Vector3& f, const Vector3& offset )
{
    // TODO apply force/torque to sphere
    force = f;
    torque = angular_velocity * 0.7;
}

}
