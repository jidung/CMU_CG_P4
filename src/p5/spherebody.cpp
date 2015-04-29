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

    gravity = Vector3::Zero(); // jd
    angular_accel = Vector3::Zero();
}

// euler step - not used
Vector3 SphereBody::step_position( real_t dt, real_t motion_damping )
{
    // Note: This function is here as a hint for an approach to take towards
    // programming RK4, you should add more functions to help you or change the
    // scheme
    // TODO return the delta in position dt in the future

    //return Vector3::Zero();

    Vector3 deltaPos;
    deltaPos = position + velocity * dt;
    velocity = (velocity + (force / mass) * dt) * (1 - motion_damping);

    return deltaPos;
}

Vector3 SphereBody::acceleration( real_t dt, Vector3 velocity)
{
//    return velocity * motion_damping * dt;
    //return (velocity + (force/mass) ) * motion_damping;
    //return (velocity * dt + (force/mass) ) * (1 - motion_damping);
    
    return (velocity * dt + (force/mass) );
}

Vector3 SphereBody::angular_acceleration( real_t dt, Vector3 angular_velocity)
{
    return (angular_velocity * dt + angular_accel);
}

// NOT USED: Euler method
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
    angular_velocity = (angular_velocity + (torque / mass) * dt) * (1 - motion_damping);

    return Vector3::Zero();
}

void SphereBody::apply_gravity( const Vector3& gravity ) 
{
    this->gravity = gravity;
    force = gravity * mass; 
}

void SphereBody::apply_force( const Vector3& f, const Vector3& offset )
{
    // TODO apply force/torque to sphere
    Vector3 to_offset = offset;

    // When the offset from the center is not zero or parallel to the force direction
    if (offset != Vector3::Zero() || dot (to_offset, f) / (length(to_offset) * length(f)) > 1) {
        //split the force into two components, linear and angular
        
        //linear force is force parallel to the offset from the center
        Vector3 normalized_offset (normalize(to_offset));
        force += dot (f, normalized_offset) * normalized_offset;
        
        //torque is perpendicular component
        torque += cross (offset, f);
        real_t i = 0.4 * mass * radius * radius;
        angular_accel = torque / i;
        
        //std::cout << " do you ever come here? " << std::endl;
    } else {
        force += f;
    }

    //force=f;
    //torque = angular_velocity * 0.7;
}

}
