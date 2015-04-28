/**
 * @file sphere.hpp
 * @brief Class defnition for Sphere.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_SPHERE_HPP_
#define _462_SCENE_SPHERE_HPP_

#include "scene/material.hpp"
#include "scene/geometry.hpp"

namespace _462 {

/**
 * A sphere, centered on its position with a certain radius.
 */
class Sphere : public Geometry
{
public:

    real_t radius;
    const Material* material;

    Sphere();
    virtual ~Sphere();
    virtual void render() const;

    // m.ji
    void make_AABB()
    {
        Vector3 pos = position;

        if (pos.x - radius < aabb.min.x)
            aabb.min.x = pos.x;
        else if (pos.x + radius > aabb.max.x)
            aabb.max.x = pos.x;

        if (pos.y - radius < aabb.min.y) 
            aabb.min.y = pos.y;
        else if (pos.y + radius > aabb.max.y)
            aabb.max.y = pos.y;

        if (pos.z - radius < aabb.min.z)
            aabb.min.z = pos.z;
        else if (pos.z + radius > aabb.max.z)
            aabb.max.z = pos.z;
    }
};

} /* _462 */

#endif /* _462_SCENE_SPHERE_HPP_ */

