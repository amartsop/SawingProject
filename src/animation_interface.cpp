#include "../include/animation_interface.h"

void AnimationInterface::initialize_animation(Scene<AnimationInterface>* scene)
{
    // Get simple shader path
    m_simple_shader_path = scene->get_simple_shader_path();
    
    // Get current path
    m_current_path = std::filesystem::current_path();
   
    // Generate background
    generate_background();

    // Generate sawing platform
    generate_sawing_platform();

    // Generate target object
    generate_target_object();
}


// Update all scene objects
void AnimationInterface::update(double real_time)
{
    // Update target object
    update_target_object();

    // Render all objects
    for (size_t i = 0; i < m_objects.size(); i++)
    {
        // Draw objects
        m_objects.at(i)->draw();
    }
}

// Generate background
void AnimationInterface::generate_background(void)
{
    obj_type::ImportedSurfaceProperties iop;
    iop.id = m_objects.size();
    // iop.pos = glm::vec3(1.5f, 0.0f, 1.41f);
    iop.pos = glm::vec3(0.0f, 0.0f, -0.1f);
    iop.euler = glm::vec3(0.0f, 0.0f, 0.0f);
    iop.shader_filename = m_simple_shader_path;
    iop.mesh_filename = (m_current_path / m_bkg_rel_mesh_name).string();
    iop.texture_filename = (m_current_path / m_bkg_rel_texture_name).string();
    
    iop.shininess = 10.0f;
    iop.draw_type = GL_STATIC_DRAW;

    m_background_id = iop.id;

    m_objects.push_back(std::make_shared<ImportedSurface>());
    m_objects.at(iop.id)->initialize_imported_surface(iop);
}

// Generate sawing platfrom
void AnimationInterface::generate_sawing_platform(void)
{
    obj_type::ImportedSurfaceProperties iop;
    iop.id = m_objects.size();
    iop.pos = glm::vec3(0.0f, 0.0f, 0.0f);
    iop.euler = glm::vec3(0.0f, 0.0f, 0.0f);
    iop.shader_filename = m_simple_shader_path;
    iop.mesh_filename = (m_current_path / m_sp_rel_mesh_name).string();
    iop.color = glm::vec3(0.8f, 0.8f, 0.8f);
    
    iop.shininess = 10.0f;
    iop.draw_type = GL_STATIC_DRAW;

    m_sawing_platform_id = iop.id;

    m_objects.push_back(std::make_shared<ImportedSurface>());
    m_objects.at(iop.id)->initialize_imported_surface(iop);
}

// Generate target object
void AnimationInterface::generate_target_object(void)
{
    obj_type::ImportedSurfaceProperties iop;
    iop.id = m_objects.size();
    // iop.pos = glm::vec3(1.5f, 0.0f, 1.41f);
    iop.pos = glm::vec3(0.0f, 0.0f, 0.0f);
    iop.euler = glm::vec3(0.0f, 0.0f, 0.0f);
    iop.shader_filename = m_simple_shader_path;
    iop.mesh_filename = (m_current_path / m_to_rel_mesh_name).string();
    iop.texture_filename = (m_current_path / m_to_rel_texture_name).string();
    
    iop.shininess = 10.0f;
    iop.draw_type = GL_STATIC_DRAW;

    m_target_object_id = iop.id;

    m_objects.push_back(std::make_shared<ImportedSurface>());
    m_objects.at(iop.id)->initialize_imported_surface(iop);
}

// Update target object
void AnimationInterface::update_target_object(void)
{
    // Get needle base pose state
    Eigen::Vector3d obj_pos, obj_eul;
    m_world_interface->get_object_pose(obj_pos, obj_eul);

    // Base position glm
    glm::vec3 pos = glm::vec3(obj_pos(0), obj_pos(1), obj_pos(2));
    glm::vec3 eul = glm::vec3(obj_eul(0), obj_eul(1), obj_eul(2));
    
    // Update position
    m_objects.at(m_target_object_id)->transform(pos, eul);
}