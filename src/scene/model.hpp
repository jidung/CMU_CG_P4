/**
 * @file model.hpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_MODEL_HPP_
#define _462_SCENE_MODEL_HPP_

#include "scene/material.hpp"
#include "scene/geometry.hpp"
#include "scene/mesh.hpp"

namespace _462 {

/**
 * A mesh of triangles.
 */
class Model : public Geometry
{
public:

    const Mesh* mesh;
    const Material* material;

    Model();
    virtual ~Model();

    virtual void render() const;

    Matrix4 mat, invMat; //m.ji

    // Axis-Aligned Bounding Box. m.ji
    struct AABB 
    {
        Vector3 min, max;
        AABB () {
            min = Vector3(9999, 9999, 9999);
            max = -min;
        }
    };
    AABB aabb;  // m.ji
    
    // m.ji
    void make_AABB()
    {
        const MeshVertex* vertices = mesh->get_vertices();
        for (unsigned int i = 0; i < mesh->num_vertices(); ++i) {
            //std::cout << mat.transform_point(vertices[i].position) << std::endl;
            Vector3 pos = mat.transform_point(vertices[i].position);

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

#endif /* _462_SCENE_MODEL_HPP_ */

