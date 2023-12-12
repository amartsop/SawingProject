#pragma once

#include <iostream>
#include <vector>

#include <glad/glad.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <GLFW/glfw3.h>
#include <indexed_model.h>

#include "obj_loader.h"


class Mesh {        

public:
    Mesh(){};

    // Assign mesh (for imported file)
    void assign(const std::string& filename, GLenum draw_type=GL_STATIC_DRAW);

    // Assign mesh (for indexed model)
    void assign(const IndexedModel& indexed_model, GLenum draw_type=GL_STATIC_DRAW);

    // Getters 
    std::vector<glm::vec3> get_obj_vertices_pos(void);
    std::vector<glm::vec2> get_obj_texture_coords(void);
    std::vector<glm::vec3> get_obj_normals(void);
    unsigned int get_obj_vertices_num(void);

    // Update mesh
    void set_obj_vertices_pos(const std::vector<glm::vec3>& vert_pos);
    
    // Draw mesh
    void draw();

    virtual ~Mesh();

private:

    void bind_mesh(void);

    enum
    {
        POSITION_VB,
        TEXCOORD_VB,
        NORMAL_VB,
        INDEX_VB,
        NUM_BUFFERS
    };

    GLuint m_vertex_array_object;
    GLuint m_vertex_array_buffers[NUM_BUFFERS];

    unsigned int m_draw_count;
    GLenum m_draw_type;
    
    // Vertices position
    std::vector<glm::vec3> m_vertices_pos;
    
    // Object handle
    OBJModel m_object; 

    // Indexed model 
    IndexedModel m_model;

    // Is model indexed flag
    bool m_is_model_indexed = 0;
};

