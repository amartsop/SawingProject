#include "line.h"

// Set object properties (basic geometry mesh)
void Line::initialize_line(const obj_type::LineProperties& iop)
{
    // Assign object's id 
    m_id = iop.id;
    
    // Assign color
    m_color = iop.color;

    // Calcualte unit orthogonal basis
    arma::dvec p1 = {iop.p1.x, iop.p1.y, iop.p1.z};
    arma::dvec p2 = {iop.p2.x, iop.p2.y, iop.p2.z};
    double line_length = arma::norm(p2-p1);
    
    // Container of ei_prime base vector
    std::vector<arma::dvec> e_prime(3);

    // Unit vector ex_prime
    e_prime.at(0) = (1.0 / line_length) * (p2 - p1);
    
    // Unit vector ey_prime
    e_prime.at(1) = EulerRotationsAnim::basic_rotation_z(M_PI / 2.0) * e_prime.at(0);
    
    // Unit vector ez_prime
    e_prime.at(2) = arma::cross(e_prime.at(0), e_prime.at(1));

    // Define rotation matrix (direction cosinesj)
    arma::dmat rot_mat = arma::zeros(3, 3);

    for (size_t i = 0; i < 3; i++)
    {
        // Base prime unit vector i
        arma::dvec ei_prime = e_prime.at(i);
        
            for (size_t j = 0; j < 3; j++)
            {
                arma::dvec ei = arma::zeros(3);
                ei(j) = 1.0;

                rot_mat(i, j) = arma::dot(ei_prime, ei);
            }
    }
    rot_mat = rot_mat.t();

    // Find euler angles from rotation matrix    
    arma::dvec euler_vec = EulerRotationsAnim::rotation_to_euler(rot_mat);
    m_euler = glm::vec3({euler_vec(0), euler_vec(1), euler_vec(2)});
    
    // Set point position
    m_pos = iop.p1;

    // Assign scale
    std::cout << iop.thickness << std::endl;
    m_scale = glm::vec3({line_length, iop.thickness, iop.thickness});

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

void Line::draw()
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