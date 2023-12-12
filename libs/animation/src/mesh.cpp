#include "../include/mesh.h"

// Assign mesh (for imported file)
void Mesh::assign(const std::string& filename, GLenum draw_type)
{
    // Draw type 
    m_draw_type = draw_type;

    // Create object handle
    m_object.initialize(filename);

    // Create indexed model
    m_model = m_object.ToIndexedModel();

    // Bind mesh
    bind_mesh();
}

// Assign mesh (for indexed model)
void Mesh::assign(const IndexedModel& indexed_model, GLenum
    draw_type)
{
    // Draw type 
    m_draw_type = draw_type;

    // Store indexed model
    m_model = indexed_model;

    // Bind mesh
    bind_mesh();

    // Model is indexed
    m_is_model_indexed = 1;
}

std::vector<glm::vec3> Mesh::get_obj_vertices_pos(void)
{
    return m_model.positions;
}

std::vector<glm::vec2> Mesh::get_obj_texture_coords(void)
{
    return m_model.texCoords;
}

std::vector<glm::vec3> Mesh::get_obj_normals(void)
{
    return m_model.normals;
}

unsigned int Mesh::get_obj_vertices_num(void)
{
    return m_model.positions.size();
}

void Mesh::set_obj_vertices_pos(const std::vector<glm::vec3>& vertPos)
{
    if (m_is_model_indexed)
    {
        m_model.positions = vertPos;
    }
    else 
    {
        m_object.setVertices(vertPos);

        // Create indexed model
        m_model = m_object.ToIndexedModel();
    }

    bind_mesh();
}


void Mesh::draw()
{
    glBindVertexArray(m_vertex_array_object);
    glDrawElements(GL_TRIANGLES, m_draw_count, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void Mesh::bind_mesh(void)
{
    m_draw_count = m_model.indices.size();
    
    glGenVertexArrays(1, &m_vertex_array_object);
    glBindVertexArray(m_vertex_array_object);
    glGenBuffers(NUM_BUFFERS, m_vertex_array_buffers);

   // Position coordinates 
    glBindBuffer(GL_ARRAY_BUFFER, m_vertex_array_buffers[POSITION_VB]);
    glBufferData(GL_ARRAY_BUFFER,
        m_model.positions.size() * sizeof(m_model.positions[0]), 
        &m_model.positions[0], m_draw_type);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    if (m_model.texCoords.size())
    {
        // Texture coordinates
        glBindBuffer(GL_ARRAY_BUFFER, m_vertex_array_buffers[TEXCOORD_VB]);
        glBufferData(GL_ARRAY_BUFFER,
            m_model.positions.size() * sizeof(m_model.texCoords[0]), 
            &m_model.texCoords[0], m_draw_type);

        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);
    }

    if (m_model.normals.size())
    {
        // Normal coordinates
        glBindBuffer(GL_ARRAY_BUFFER, m_vertex_array_buffers[NORMAL_VB]);
        glBufferData(GL_ARRAY_BUFFER,
            m_model.normals.size() * sizeof(m_model.normals[0]), 
            &m_model.normals[0], m_draw_type);

        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);
    }

    // Indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vertex_array_buffers[INDEX_VB]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        m_model.indices.size() * sizeof(m_model.indices[0]), 
        &m_model.indices[0], m_draw_type);

    //Debind
    glBindVertexArray(0);
}


Mesh::~Mesh()
{
    glDeleteVertexArrays(1, &m_vertex_array_object);

    for (uint i = 0; i < NUM_BUFFERS; i++)
    {
        glDeleteBuffers(1, &m_vertex_array_buffers[i]);
    }
}