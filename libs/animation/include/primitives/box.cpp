#include "box.h"

// Set object properties (basic geometry mesh)
void Box::initialize_box(const obj_type::BoxProperties& iop)
{
    // Assign object's id 
    m_id = iop.id;

    // Assign object's initial position and orientation
    m_pos = iop.pos; 
    m_euler = iop.euler;

    // Assign color
    m_color = iop.color;

    // Assign scale
    m_scale = iop.dimensions;

    // Save shader to member variable
    m_shader_abs_path = m_absolute_path + m_shader_rel_path;
    m_shader.initialize(m_shader_abs_path);

    // Assign shiness and specular
    m_shininess = iop.shininess;
    m_specular = iop.specular;

    // Wireframe flag
    m_wireframe_flag = iop.wireframe;

    // Set mesh filename
    m_mesh_abs_path = m_absolute_path + m_point_rel_path;

    // Generate object's mesh
    m_mesh.assign(m_mesh_abs_path, iop.draw_type);

    // Set transform scale
    m_transform.setScale(m_scale);
}

void Box::draw(void)
{
    // Handle transform 
    m_transform.setPos(m_pos);
    m_transform.setRot(m_euler);

    // Draw handle
    glm::mat4 model = m_transform.getModel();
    m_shader.set_mat4("model", model);

    // Bind texture and set color
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