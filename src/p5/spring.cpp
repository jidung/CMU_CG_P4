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
   
    // offsets seem to be in world coordinate.
    /*
    Vector3 force;
    Vector3 displacement = (body1->position - body1_offset);

    force = - constant * displacement * equilibrium - damping * displacement / dt;
    force *= dt;

    // std::cout << dot (force, body2_offset) << std::endl;
    body1->apply_force(force, body1_offset);
*/

    Vector3 force;

    Vector3 offset1 = body1->orientation * body1_offset;
    Vector3 offset2 = body2->orientation * body2_offset;

    Vector3 direction = (body1->position + offset1) - (body2->position + offset2);
    Vector3 displacement = (length(direction) - equilibrium) * normalize(direction);
    force = - constant * displacement - damping * displacement * dt;

    body1->apply_force(force, offset1);
    
    direction = (body2->position + offset1) - (body1->position + offset2);
    displacement = (length(direction) - equilibrium) * normalize(direction);
    force = - constant * displacement - damping * displacement / dt;

    body2->apply_force(force, offset2);

}

}


