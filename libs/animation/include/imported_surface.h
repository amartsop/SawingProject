#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include "shader.h"
#include "mesh.h"
#include "texture.h"
#include "transform.h"
#include "object.h"
#include "object_type.hpp"

class ImportedSurface : public Object
{
    
public:

    // Initialize imported surface
    void initialize_imported_surface(const obj_type::ImportedSurfaceProperties& props);

public:

    // Draw object 
    void draw(void);

    // Transform
    inline void transform(const glm::vec3& pos, const glm::vec3& euler)
        { m_pos = pos; m_euler = euler; }

    // Get object vertices
    std::vector<glm::vec3> get_object_vertices_positions(void) { return
        m_mesh.get_obj_vertices_pos(); }

    // Set object vertices
    void set_object_vertices_positions(const
        std::vector<glm::vec3>& new_vertices) { m_mesh.set_obj_vertices_pos(new_vertices); };

private:

    // Object ID
    unsigned int m_id = 0;

    // Object position
    glm::vec3 m_pos = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 m_euler = glm::vec3(0.0f, 0.0f, 0.0f);

    // Scale 
    glm::vec3 m_scale;

    // Object shader 
    Shader m_shader;

    // Object mesh
    Mesh m_mesh;

    // Object texture
    Texture m_texture;

    // Object transform
    Transform m_transform;

    // Custom color flag
    bool m_color_flag = false;

    // Custom color
    glm::vec3 m_color;

    // Material shininess 
    float m_shininess;

    // Material shininess 
    glm::vec3 m_specular;

    // Wireframe flag
    bool m_wireframe_flag;
};

