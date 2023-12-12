#pragma once 
#include <iostream>

#include <glad/glad.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include "shader.h"
#include "mesh.h"
#include "texture.h"
#include "transform.h"
#include "object_type.hpp"

class Object
{
    
public:

    // Set object properties (basic geometry mesh)
    Object() {};
    virtual ~Object() {};

    public:

    // Initialize imported surface
    virtual void initialize_imported_surface(const
        obj_type::ImportedSurfaceProperties& props) {};

    // Initialize indexed surface
    virtual void initialize_indexed_surface(const
        obj_type::IndexedSurfaceProperties& props) {};

    // Initialize box
    virtual void initialize_box(const obj_type::BoxProperties& props) {};

    // Initialize point
    virtual void initialize_point(const obj_type::PointProperties& props) {};

    // Initialize line
    virtual void initialize_line(const obj_type::LineProperties& props) {};

    // Initialze sphere
    virtual void initialize_sphere(const obj_type::SphereProperties& props) {};

    // Initialze plane
    virtual void initialize_plane(const obj_type::PlaneProperties& props) {};

    // Get object vertices
    virtual std::vector<glm::vec3> get_object_vertices_positions(void) {
        std::vector<glm::vec3> a; return a; };

    // Set object vertices
    virtual void set_object_vertices_positions(const
        std::vector<glm::vec3>& new_vertices) { };

public:

    // Draw object 
    virtual void draw(void) = 0;

    // Transform
    virtual void transform(const glm::vec3& pos, const glm::vec3& euler) = 0;
};