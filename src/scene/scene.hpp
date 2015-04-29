/**
 * @file scene.hpp
 * @brief Class definitions for scenes.
 *
 * @author Eric Butler (edbutler)
 * @author Kristin Siu (kasiu)
 */

#ifndef _462_SCENE_SCENE_HPP_
#define _462_SCENE_SCENE_HPP_

#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include "math/matrix.hpp"
#include "math/camera.hpp"
#include "scene/material.hpp"
#include "p5/physics.hpp"
#include "scene/mesh.hpp"
#include "scene/geometry.hpp"
#include <string>
#include <vector>
#include <map>
#include <cfloat>

namespace _462 {

struct PointLight
{
    struct Attenuation
    {
        real_t constant;
        real_t linear;
        real_t quadratic;
    };

    PointLight();

    Color3 get_color(real_t distance) const;

    // The position of the light, relative to world origin.
    Vector3 position;
    // The color of the light (both diffuse and specular)
    Color3 color;
    // attenuation
    Attenuation attenuation;
};

/**
 * The container class for information used to render a scene composed of
 * Geometries.
 */
class Scene
{
public:

    /// the camera
    Camera camera;
    /// the background color
    Color3 background_color;
    /// the amibient light of the scene
    Color3 ambient_light;
    /// the refraction index of air
    real_t refractive_index;
    
    /// Creates a new empty scene.
    Scene();

    /// Destroys this scene. Invokes delete on everything in geometries.
    ~Scene();

    // accessor functions
    Geometry* const* get_geometries() const;
    size_t num_geometries() const;
    const PointLight* get_lights() const;
    size_t num_lights() const;
    Material* const* get_materials() const;
    size_t num_materials() const;
    Mesh* const* get_meshes() const;
    size_t num_meshes() const;
    Physics* get_physics();

    /// Clears the scene, and invokes delete on everything in geometries,
    //bodies, and springs.
    void reset();

    // functions to add things to the scene
    // all pointers are deleted by the scene upon scene deconstruction.
    void add_geometry( Geometry* g );
    void add_material( Material* m );
    void add_mesh( Mesh* m );
    void add_light( const PointLight& l );

    void update( real_t dt );

private:

    typedef std::vector< PointLight > PointLightList;
    typedef std::vector< Material* > MaterialList;
    typedef std::vector< Mesh* > MeshList;
    typedef std::vector< Geometry* > GeometryList;

    // list of all lights in the scene
    PointLightList point_lights;
    // all materials used by geometries
    MaterialList materials;
    // all meshes used by models
    MeshList meshes;
    // list of all geometries. deleted in dctor, so should be allocated on heap.
    GeometryList geometries;
    // the physics engine
    Physics phys;

    struct octree_node {
        Vector3 max;
        Vector3 min;
        Geometry* geometry;
        int child_idx;

        octree_node () {
            min = Vector3 (99999, 99999, 99999);
            max = -min;
            geometry = NULL;
            child_idx = -1;
        }
    };

    class Octree 
    {
    private:
        std::vector< octree_node > octree_array;
    public:
        void construct_octree (size_t num_geometries, Geometry* geometries) {

            size_t idx = 0;
            octree_node node;
            
            for (size_t i = 0; i < num_geometries; ++i) {

                if (node.min.x > geometries[i].aabb.min.x)
                    node.min.x = geometries[i].aabb.min.x;
                else if (octree_array[idx].max.x < geometries[i].aabb.max.x)
                    node.max.x = geometries[i].aabb.max.x;

                if (node.min.y > geometries[i].aabb.min.y)
                    node.min.y = geometries[i].aabb.min.y;
                else if (node.max.y < geometries[i].aabb.max.y)
                    node.max.y = geometries[i].aabb.max.y;
                
                if (node.min.z > geometries[i].aabb.min.z)
                    node.min.z = geometries[i].aabb.min.z;
                else if (node.max.z < geometries[i].aabb.max.z)
                    node.max.z = geometries[i].aabb.max.z;
            }
            
            for (size_t i = 0; i < num_geometries; ++i) {
                 
            }    
        }
    };


private:

    // no meaningful assignment or copy
    Scene(const Scene&);
    Scene& operator=(const Scene&);

};

} /* _462 */

#endif /* _462_SCENE_SCENE_HPP_ */

