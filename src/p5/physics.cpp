#include "p5/physics.hpp"

namespace _462 {

Physics::Physics()
{
    reset();
}

Physics::~Physics()
{
    reset();
}

void Physics::step( real_t dt )
{
    bool didCollide = false;
    elapsed_time += dt;

    for ( SphereList::iterator i = spheres.begin(); i != spheres.end(); i++ ) {
        (*i)->force = Vector3::Zero();
        (*i)->torque = Vector3::Zero();
    }

    for ( SpringList::iterator j = springs.begin(); j != springs.end(); j++)
        (*j)->step(dt);

    for ( SphereList::iterator i = spheres.begin(); i != spheres.end(); i++ ) {
        
        for ( SphereList::iterator j = spheres.begin(); j != spheres.end(); j++ ) {
            if ( *i != *j ) // don't check collision with itself
                didCollide |= collides ( *(*i), *(*j), collision_damping );
        }
    
        for ( PlaneList::iterator j = planes.begin(); j != planes.end(); j++ ) {
            didCollide |= collides ( *(*i), *(*j), collision_damping );
        }
        
        for ( TriangleList::iterator j = triangles.begin(); j != triangles.end(); j++ ) 
        {
            bool didTriangleCollide = collides ( *(*i), *(*j), collision_damping );

            if ( ( (*j)->id == 999 || (*j)->id == 1000 ) && didTriangleCollide )
            {
                isGameOver = true;
                std::cout << "GAME OVER!!!" << std::endl;
                std::cout << "Elapsed Time: " << elapsed_time << " seconds (developer's best: 22.2367)" << std::endl;
                std::cout << "Press ESC to quit" << std::endl;
            }

            didCollide |= didTriangleCollide;
        }

        for ( ModelList::iterator j = models.begin(); j != models.end(); j++ ) {
            (*j)->model->update_matrices_and_aabb();
            didCollide |= collides ( *(*i), *(*j), collision_damping );
            (*j)->model->position = (*j)->position;
        }
       
        //if (didCollide) 
        //    std::cout << "collision detected" << std::endl;

        // RK4 integration of velocity, position, angular velocity, orientation
        Vector3 initial_vel = (*i)->velocity;
       
        // dont' apply force only when it's colliding with something 
        // and velocity is zero
        if (!didCollide || initial_vel != Vector3::Zero())
            (*i)->apply_force ( gravity*(*i)->mass, Vector3::Zero() );
       
        Vector3 initial_pos = (*i)->position;
      
        (*i)->velocity = rk4(&Physics::evaluate, initial_vel, (*i)->force/(*i)->mass, dt) * (1 - motion_damping);
        (*i)->position = rk4(&Physics::evaluate, initial_pos, initial_vel, dt);
        
	/* expanded version of RK4 - not used
        Vector3 k1, k2, k3, k4;
        Vector3 k1_p, k2_p, k3_p, k4_p; // for position

        k1   = (*i)->acceleration (dt, initial_vel);
        k1_p = (initial_vel);
        k2   = (*i)->acceleration (dt*0.5, initial_vel + k1*dt*0.5);
        k2_p = (initial_vel + k1*dt*0.5); 
        k3   = (*i)->acceleration (dt*0.5, initial_vel + k2*dt*0.5);
        k3_p = (initial_vel + k2*dt*0.5); 
        k4   = (*i)->acceleration (dt, initial_vel + k3*dt);
        k4_p = (initial_vel + k3*dt); 

        (*i)->velocity = initial_vel + (k1*(1.0/6.0) + k2*(1.0/3.0) + k3*(1.0/3.0) + k4*(1.0/6.0)) * dt;
        (*i)->position = initial_pos + (k1_p*(1.0/6.0) + k2_p*(1.0/3.0) + k3_p*(1.0/3.0) + k4_p*(1.0/6.0)) * dt;
	*/


        // Expanded version of RK4 for angular velocity and angular orientation integration
        Vector3 initial_avel = (*i)->angular_velocity;
        Vector3 k1_a, k2_a, k3_a, k4_a; // for angular velocity
        Vector3 k1_s, k2_s, k3_s, k4_s; // for orientation
        k1_a = (*i)->angular_acceleration (dt, initial_avel);
        k1_s = (initial_avel);
        k2_a = (*i)->angular_acceleration (dt*0.5, initial_avel + k1_a*dt*0.5);
        k2_s = (initial_avel + k1_a*dt*0.5);
        k3_a = (*i)->angular_acceleration (dt*0.5, initial_avel + k2_a*dt*0.5);
        k3_s = (initial_avel + k2_a*dt*0.5);
        k4_a = (*i)->angular_acceleration (dt, initial_avel + k3_a*dt);
        k4_s = (initial_avel + k3_a*dt);
        
        (*i)->angular_velocity = 
            initial_avel + (k1_a*(1.0/6.0) + k2_a*(1.0/3.0) + k3_a*(1.0/3.0) + k4_a*(1.0/6.0)) * dt;
	(*i)->angular_velocity *= (1 - rotation_damping);       

	// to get orientation 
        Vector3 spin = (k1_s*(1.0/6.0) + k2_s*(1.0/3.0) + k3_s*(1.0/3.0) + k4_s*(1.0/6.0)) * dt;
        Vector3 axis = normalize( spin ); 
        real_t magnitude = length( spin );
        
        /*
        (*i)->angular_velocity = rk4(&Physics::evaluate, initial_avel, (*i)->angular_accel, dt);
        Vector3 spin = rk4(&Physics::evaluate, (*i)->orientation?? <- this can't be used
        Vector3 axis = normalize( spin ); 
        real_t magnitude = length( spin );
        */

        // don't do this when rotation magnitude is zero
        if (magnitude != 0) { 
            Quaternion delta_orientation( axis, magnitude );
            (*i)->orientation = normalize( (*i)->orientation * delta_orientation );
        } 
        
	/* Euler method
        (*i)->position = (*i)->step_position(dt, 0.001);
        (*i)->step_orientation(dt, 0);
        */
       
        // update graphical representation
        (*i)->sphere->position = (*i)->position;
        (*i)->sphere->orientation = (*i)->orientation;
    }

    // TODO step the world forward by dt. Need to detect collisions, apply
    // forces, and integrate positions and orientations.
    //
    // Note: put RK4 here, not in any of the physics bodies
    //
    // Must use the functions that you implemented
    //
    // Note, when you change the position/orientation of a physics object,
    // change the position/orientation of the graphical object that represents
    // it
}
/*
void Physics::rk4(Vector3(*f)(Vector3, Vector3), real_t dt, Vector3 x, Vector3 t)
{
    Vector3 k1 = dt * f(x, t),
            k2 = dt * f(x + k1 * 0.5, t + dt * 0.5),
            k3 = dt * f(x + k2 * 0.5, t + dt * 0.5),
            k4 = dt * f(x + k3, y + dt);

    return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;

}
*/

void Physics::add_sphere( SphereBody* b )
{
    spheres.push_back( b );
}

size_t Physics::num_spheres() const
{
    return spheres.size();
}

void Physics::add_plane( PlaneBody* p )
{
    planes.push_back( p );
}

size_t Physics::num_planes() const
{
    return planes.size();
}

void Physics::add_triangle( TriangleBody* t )
{
    triangles.push_back( t );
}

size_t Physics::num_triangles() const
{
    return triangles.size();
}

void Physics::add_model( ModelBody* m )
{
    models.push_back( m );
}

size_t Physics::num_models() const
{
    return models.size();
}

void Physics::add_spring( Spring* s )
{
    springs.push_back( s );
}

size_t Physics::num_springs() const
{
    return springs.size();
}

void Physics::reset()
{
    for ( SphereList::iterator i = spheres.begin(); i != spheres.end(); i++ ) {
        delete *i;
    }
    for ( PlaneList::iterator i = planes.begin(); i != planes.end(); i++ ) {
        delete *i;
    }
    for ( TriangleList::iterator i = triangles.begin(); i != triangles.end(); i++ ) {
        delete *i;
    }
    for ( ModelList::iterator i = models.begin(); i != models.end(); i++) {
        delete *i;
    }
    for ( SpringList::iterator i = springs.begin(); i != springs.end(); i++ ) {
        delete *i;
    }

    spheres.clear();
    planes.clear();
    triangles.clear();
    springs.clear();
    models.clear();

    gravity = Vector3::Zero();
	collision_damping = 0.0;

    /* m.ji */
    motion_damping = 0.0;
    rotation_damping = 0.0;
    isGameOver = false;
    elapsed_time = 0.0;
}

}
