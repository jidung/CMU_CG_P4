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
    for ( SphereList::iterator i = spheres.begin(); i != spheres.end(); i++ ) {

        for ( SphereList::iterator j = spheres.begin(); j != spheres.end(); j++ ) {
            if ( *i != *j )
                collides ( *(*i), *(*j), 0.993 );
        }
    
        for ( PlaneList::iterator j = planes.begin(); j != planes.end(); j++ ) {
            collides ( *(*i), *(*j), 0.99 );
        }
        
        for ( TriangleList::iterator j = triangles.begin(); j != triangles.end(); j++ ) {
            if (collides ( *(*i), *(*j), 0.991 )) {}
//                std::cout << "triangle" << std::endl;
        }

        for ( ModelList::iterator j = models.begin(); j != models.end(); j++ ) {
            
           if ( (*j)->model->position != (*j)->position )
               std::cout << "different" << std::endl;
            collides ( *(*i), *(*j), 1.0 );
//            std::cout << (*j)->position << std::endl;
        }
        
        (*i)->apply_force ( gravity, Vector3::Zero() );
        (*i)->position = (*i)->step_position(dt, 0.9998);
        (*i)->sphere->position = (*i)->position;
    }

//    std::cout << planes.size() << std::endl;
//    std::cout << spheres.size() << std::endl;

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

    // std::cout << scene.num_geometries() << std::endl;
}

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
}

}
