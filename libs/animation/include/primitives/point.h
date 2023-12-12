#pragma once

#include <iostream>


#include "../filesystem.h"
#include "../object.h"
#include "../include/object_type.hpp"

class Point : public Object
{

public:

    // Initialize point
    void initialize_point(const obj_type::PointProperties& props);
    
    // Get object vertices
    std::vector<glm::vec3> get_object_vertices_positions(void) { return
        m_mesh.get_obj_vertices_pos(); }
 
    // Set object vertices
    void set_object_vertices_positions(const
        std::vector<glm::vec3>& new_vertices) { m_mesh.set_obj_vertices_pos(new_vertices); };

public:

    // Draw object 
    void draw(void);

    // Transform
    inline void transform(const glm::vec3& pos, const glm::vec3& euler)
        { m_pos = pos; m_euler = euler; }

private:
    
    // Filepath
    std::string m_absolute_path = FileSystem::get_absolute_path();

    // Shader relative and absolute paths
    std::string m_shader_rel_path = "/share/simpleShader";
    std::string m_shader_abs_path;

    // Mesh relative filename
    std::string m_point_rel_path = "/include/primitives/primitive_objects/sphere.obj";
    std::string m_mesh_abs_path;

private:
    // Object ID
    unsigned int m_id = 0;

    // Object position
    glm::vec3 m_pos = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 m_euler = glm::vec3(0.0f, 0.0f, 0.0f);

    // Custom color
    glm::vec3 m_color = glm::vec3({0.0, 0.0, 0.0});

    // Scale 
    glm::vec3 m_scale;

    // Object shader 
    Shader m_shader;

    // Object mesh
    Mesh m_mesh;

    // Object transform
    Transform m_transform;

    // Custom color flag
    bool m_color_flag = true;

    // Material shininess 
    float m_shininess;

    // Material shininess 
    glm::vec3 m_specular;

    // Wireframe flag
    bool m_wireframe_flag;
};
