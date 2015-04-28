/**
 * @file triangle.hpp
 * @brief Class definition for Triangle.
 *
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_TRIANGLE_HPP_
#define _462_SCENE_TRIANGLE_HPP_

#include "scene/material.hpp"
#include "scene/geometry.hpp"

namespace _462 {

/**
 * a triangle geometry.
 * Triangles consist of 3 vertices. Each vertex has its own position, normal,
 * texture coordinate, and material. These should all be interpolated to get
 * values in the middle of the triangle.
 * These values are all in local space, so it must still be transformed by
 * the Geometry's position, orientation, and scale.
 */
class Triangle : public Geometry
{
public:

    struct Vertex
    {
        // note that position and normal are in local space
        Vector3 position;
        Vector3 normal;
        Vector2 tex_coord;
        const Material* material;
    };

    // the triangle's vertices, in CCW order
    Vertex vertices[3];

    Triangle();
    virtual ~Triangle();
    virtual void render() const;

    // m.ji
    void make_AABB()
    {
        for (unsigned int i = 0; i < 3; ++i) {
            //std::cout << mat.transform_point(vertices[i].position) << std::endl;
            Vector3 pos = vertices[i].position;

            if (pos.x < aabb.min.x)
                aabb.min.x = pos.x;
            else if (pos.x > aabb.max.x)
                aabb.max.x = pos.x;

            if (pos.y < aabb.min.y) 
                aabb.min.y = pos.y;
            else if (pos.y > aabb.max.y)
                aabb.max.y = pos.y;

            if (pos.z < aabb.min.z)
                aabb.min.z = pos.z;
            else if (pos.z > aabb.max.z)
                aabb.max.z = pos.z;
        }
    }
};


} /* _462 */

#endif /* _462_SCENE_TRIANGLE_HPP_ */

