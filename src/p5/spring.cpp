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
    
    force = - constant * (body1->position - body1_offset)
        - damping * (body1->position - body1_offset) / dt;
    // should be fixed

    force *= dt;

    // std::cout << dot (force, body2_offset) << std::endl;
    body1->apply_force(force, body1_offset);

}

}


