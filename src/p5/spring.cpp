#include "math/math.hpp"
#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include "math/matrix.hpp"
#include "p5/spring.hpp"
#include "p5/body.hpp"
#include "p5/spherebody.hpp"
#include <iostream>

namespace _462 {

Spring::Spring()
{
    body1_offset = Vector3::Zero();
    body2_offset = Vector3::Zero();
    damping = 0.0;
}

void Spring::step( real_t dt )
{
    // TODO apply forces to attached bodies
   
    Vector3 force;

    Vector3 offset1 = body1->orientation * body1_offset;
    Vector3 offset2 = body2->orientation * body2_offset;

    Vector3 direction = (body1->position + offset1) - (body2->position + offset2);

    Vector3 normalized_dir = normalize(direction);

    Vector3 velocity_component = dot(body1->velocity, normalized_dir) * normalized_dir;
    Vector3 displacement = (length(direction) - equilibrium) * normalized_dir;
    force = - constant * displacement - damping * velocity_component / dt;

    body1->apply_force(force, offset1);

    
    direction = (body2->position + offset1) - (body1->position + offset2);
    normalized_dir = normalize(direction);
    velocity_component = dot(body2->velocity, normalized_dir) * normalized_dir;
    displacement = (length(direction) - equilibrium) * normalize(direction);
    force = - constant * displacement - damping * velocity_component / dt;

    body2->apply_force(force, offset2);

}

}


