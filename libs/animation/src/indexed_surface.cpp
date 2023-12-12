#include <indexed_surface.h>

// Set object properties (basic geometry mesh)
void IndexedSurface::initialize_indexed_surface(const
    obj_type::IndexedSurfaceProperties& iop)
{
    // Assign object's id 
    m_id = iop.id;

    // Assign object's initial position and orientation
    m_pos = iop.pos; m_euler = iop.euler;

    // Set scale
    m_scale = iop.scale;

    // Save shader to member variable
    m_shader.initialize(iop.shader_filename);

    // Check for textrue
    if (iop.texture_filename.empty())
    {
        m_color_flag = true;
        m_color = iop.color;
    }
    else 
    {
        // Generate object's texture
        m_texture.assign(iop.texture_filename);
    }

    // Assign shiness and specular
    m_shininess = iop.shininess;
    m_specular = iop.specular;

    // Wireframe flag
    m_wireframe_flag = iop.wireframe;

    // Generate object's mesh
    m_mesh.assign(iop.indexed_model, iop.draw_type);
}

// Draw importes surface
void IndexedSurface::draw()
{
    // Handle transform 
    m_transform.setPos(m_pos);
    m_transform.setRot(m_euler);
    m_transform.setScale(m_scale);

    // Draw handle
    glm::mat4 model = m_transform.getModel();
    m_shader.set_mat4("model", model);

    // Bind texture and set color
    m_texture.bind(0);
    m_shader.set_bool("monochrome", m_color_flag);
    m_shader.set_vec3("custom_color", m_color);

    // Set material proporties
    m_shader.set_vec3("material.specular", m_specular);
    m_shader.set_float("material.shininess", m_shininess);

    // Check for wireframe 
    GLenum draw_type;
    m_wireframe_flag ?  (draw_type = GL_LINE) : (draw_type = GL_FILL);
    glPolygonMode(GL_FRONT_AND_BACK, draw_type);

    // Draw mesh
    m_mesh.draw();
}