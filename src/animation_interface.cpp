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


// // Generate tissue mesh
// void AnimationInterface::generate_tissue_mesh(void)
// {
//     // Read tissue mesh
//     obj_type::IndexedSurfaceProperties iop;
//     iop.id = m_objects.size();
//     iop.pos = glm::vec3(0.0f, 0.0f, 0.0f);
//     iop.euler = glm::vec3(0.0f, 0.0f, 0.0f);
//     iop.shader_filename = m_simple_shader_path;
//     // iop.color = glm::vec3(1.0, 0.0, 0.0);

//     // Get tissue mesh
//     Eigen::MatrixXd tissue_vertices;
//     m_world_interface->get_tissue_visual_vertices(tissue_vertices);
    
//     Eigen::MatrixXi tissue_faces;
//     m_world_interface->get_tissue_visual_faces(tissue_faces);
    
//     // Create indexed model
//     iop.indexed_model.positions.resize(tissue_vertices.rows());
//     iop.indexed_model.indices.resize(3 * tissue_faces.rows());

//     for (size_t i = 0 ; i < iop.indexed_model.positions.size(); i++)
//     {
//         iop.indexed_model.positions.at(i) = glm::vec3(tissue_vertices(i, 0), 
//             tissue_vertices(i, 1), tissue_vertices(i, 2));
//     }

//     for (size_t i = 0 ; i < tissue_faces.rows(); i++)
//     {
//         iop.indexed_model.indices.at(3 * i) = tissue_faces(i, 0);
//         iop.indexed_model.indices.at(3 * i + 1) = tissue_faces(i, 1);
//         iop.indexed_model.indices.at(3 * i + 2) = tissue_faces(i, 2);
//     }

//     iop.shininess = 10.0f;
//     iop.draw_type = GL_STATIC_DRAW;
//     iop.wireframe = true;
//     m_tissue_model_id = iop.id;

//     m_objects.push_back(std::make_shared<IndexedSurface>());
//     m_objects.at(iop.id)->initialize_indexed_surface(iop);
// }

// // Generate needle points
// void AnimationInterface::generate_needle(void)
// {
//     // Relative nedle
//     std::string abs_needle_path = m_current_path / m_needle_rel_mesh_name;
    
//     // Read needle file     
//     igl::readOBJ(abs_needle_path, m_init_needle_vertices, m_init_needle_faces);

//     // Get needle base pose state
//     Eigen::Vector3d needle_pos, needle_eul;
//     m_world_interface->get_needle_base_state(needle_pos, needle_eul);

//     obj_type::ImportedSurfaceProperties iop;
//     iop.id = m_objects.size();
//     iop.pos = glm::vec3(needle_pos(0), needle_pos(1), needle_pos(2));
//     iop.euler = glm::vec3(needle_eul(0), needle_eul(1), needle_eul(2));
//     iop.shader_filename = m_simple_shader_path;
//     iop.mesh_filename = (m_current_path / m_needle_rel_mesh_name).string();
//     iop.texture_filename = (m_current_path / m_needle_rel_texture_name).string();
    
//     iop.shininess = 10.0f;
//     iop.draw_type = GL_STATIC_DRAW;

//     m_needle_id = iop.id;

//     m_objects.push_back(std::make_shared<ImportedSurface>());
//     m_objects.at(iop.id)->initialize_imported_surface(iop);
// }


// // Update needle 
// void AnimationInterface::update_needle(void)
// {
//     if (m_needle_visual)
//     {
//         // Update visual needle mesh
//         update_needle_visual_mesh();
    
//         // Update needle base frame
//         if (m_base_render) { update_needle_base_frame(); }
    
//         // Update handle
//         if (m_handle_render) { update_handle(); }
//     }
//     else 
//     {
//         update_needle_points();
//     }
// }

// // Update needle visual mesh
// void AnimationInterface::update_needle_visual_mesh(void)
// {
//     // New needle vertices
//     std::vector<glm::vec3> new_vertices;

//     // Update needle points
//     for (size_t i = 0; i < m_init_needle_vertices.rows(); i++)
//     {
//         // Get new needle vertex
//         Eigen::Vector3d rop_F_F;
//         m_world_interface->get_needle_visual_vertices(
//             m_init_needle_vertices.row(i).transpose(), rop_F_F);

//         new_vertices.push_back(glm::vec3(rop_F_F(0), rop_F_F(1), rop_F_F(2)));
//     }
    
//     m_objects.at(m_needle_id)->set_object_vertices_positions(new_vertices);
    
// }

// // Update tissue vertices
// void AnimationInterface::update_tissue_vertices()
// {
//     // Get tissue vertices
//     Eigen::MatrixXd tissue_vertices;
//     m_world_interface->get_tissue_visual_vertices(tissue_vertices);

//     // Initialize new tissue vertices
//     std::vector<glm::vec3> new_tissue_vertices(tissue_vertices.rows());

//     // Setup new vertices
//     for (size_t i = 0; i < tissue_vertices.rows(); i++)
//     {
//         new_tissue_vertices.at(i) = {tissue_vertices(i, 0),
//             tissue_vertices(i, 1), tissue_vertices(i, 2)};
//     }

//     // Send vertices to renderer
//     m_objects.at(m_tissue_model_id)->set_object_vertices_positions(new_tissue_vertices);
// }

// // Generate boundary points
// void AnimationInterface::generate_boundary_points(void)
// {
//     // Get in tissue needle points int
//     Eigen::MatrixXd tissue_boundary_pts;
//     m_world_interface->get_boundary_points(tissue_boundary_pts);

//     // Check which points are active
//     m_boundary_pts_ids.resize(0);

//     // Generate boundary points
//     for (size_t i = 0; i < tissue_boundary_pts.rows(); i++)
//     {
//         // Needle point
//         Eigen::Vector3d bnd_pt = tissue_boundary_pts.row(i).transpose();
        
//         obj_type::PointProperties iop;
//         iop.id = m_objects.size();
//         iop.pos = glm::vec3({bnd_pt(0), bnd_pt(1), bnd_pt(2)});
//         iop.draw_type = GL_STATIC_DRAW;
//         iop.size = 1.5e-3;
//         iop.color = glm::vec3(1.0, 0.0, 0.0);

//         iop.wireframe = false;

//         m_boundary_pts_ids.push_back(iop.id);
//         m_objects.push_back(std::make_shared<Point>());
//         m_objects.at(iop.id)->initialize_point(iop);
//     }
// } 

// // Generate needle base frame
// void AnimationInterface::generate_needle_base_frame(void)
// {
//     // Relative nedle
//     std::string abs_needle_path = m_current_path / m_frame_rel_mesh_name;

//     // Get needle base pose state
//     Eigen::Vector3d needle_pos, needle_eul;
//     m_world_interface->get_needle_base_state(needle_pos, needle_eul);

//     obj_type::ImportedSurfaceProperties iop;
//     iop.id = m_objects.size();
//     iop.pos = glm::vec3(needle_pos(0), needle_pos(1), needle_pos(2));
//     iop.euler = glm::vec3(needle_eul(0), needle_eul(1), needle_eul(2));
//     iop.shader_filename = m_simple_shader_path;
//     iop.mesh_filename = (m_current_path / m_frame_rel_mesh_name).string();
//     iop.texture_filename = (m_current_path / m_frame_rel_texture_name).string();
//     iop.scale = glm::vec3(0.01, 0.01, 0.01);
    
//     iop.shininess = 10.0f;
//     iop.draw_type = GL_STATIC_DRAW;

//     m_needle_frame_base_id = iop.id;

//     m_objects.push_back(std::make_shared<ImportedSurface>());
//     m_objects.at(iop.id)->initialize_imported_surface(iop);
// }

// // Update needle base frame
// void AnimationInterface::update_needle_base_frame(void)
// {
//     // Get needle base pose state
//     Eigen::Vector3d needle_pos, needle_eul;
//     m_world_interface->get_needle_base_state(needle_pos, needle_eul);

//     // Base position glm
//     glm::vec3 pos = glm::vec3(needle_pos(0), needle_pos(1), needle_pos(2));
//     glm::vec3 eul = glm::vec3(needle_eul(0), needle_eul(1), needle_eul(2));
    
//     // Update position
//     m_objects.at(m_needle_frame_base_id)->transform(pos, eul);
// }

// // Generate handle
// void AnimationInterface::generate_handle(void)
// {
//     // Relative nedle
//     std::string abs_needle_path = m_current_path / m_handle_rel_mesh_name;

//     // Get needle base pose state
//     Eigen::Vector3d needle_pos, needle_eul;
//     m_world_interface->get_needle_base_state(needle_pos, needle_eul);

//     obj_type::ImportedSurfaceProperties iop;
//     iop.id = m_objects.size();
//     iop.pos = glm::vec3(needle_pos(0), needle_pos(1), needle_pos(2));
//     iop.euler = glm::vec3(needle_eul(0), needle_eul(1), needle_eul(2));
//     iop.shader_filename = m_simple_shader_path;
//     iop.mesh_filename = (m_current_path / m_handle_rel_mesh_name).string();
//     iop.texture_filename = (m_current_path / m_handle_rel_texture_name).string();
    
//     iop.shininess = 10.0f;
//     iop.draw_type = GL_STATIC_DRAW;

//     m_handle_id = iop.id;

//     m_objects.push_back(std::make_shared<ImportedSurface>());
//     m_objects.at(iop.id)->initialize_imported_surface(iop);
// }

// // Update handle
// void AnimationInterface::update_handle(void)
// {
//     // Get needle base pose state
//     Eigen::Vector3d needle_pos, needle_eul;
//     m_world_interface->get_needle_base_state(needle_pos, needle_eul);

//     // Base position glm
//     glm::vec3 pos = glm::vec3(needle_pos(0), needle_pos(1), needle_pos(2));
//     glm::vec3 eul = glm::vec3(needle_eul(0), needle_eul(1), needle_eul(2));
    
//     // Update position
//     m_objects.at(m_handle_id)->transform(pos, eul);
// }


// // Generate needle interaction points
// void AnimationInterface::generate_interaction_points(void)
// {
//     // // Get in tissue needle points int
//     // std::vector<int> in_tissue_needle_pts_indices;
//     // m_world_interface->get_in_tissue_needle_points(in_tissue_needle_pts_indices);

//     // // Check which points are active
//     // m_interaction_pts_ids.resize(0);

//     // // Generate needle interaction poitns
//     // for (size_t i = 0; i < in_tissue_needle_pts_indices.size(); i++)
//     // {
//     //     // Get idx i
//     //     int idx_i = in_tissue_needle_pts_indices.at(i);

//     //     // Needle point
//     //     Eigen::Vector3d needle_pt = m_needle_pts_pos.row(idx_i).transpose();

//     //     obj_type::PointProperties iop;
//     //     iop.id = m_objects.size();
//     //     iop.pos = glm::vec3({needle_pt(0), needle_pt(1), needle_pt(2)});
//     //     iop.draw_type = GL_STATIC_DRAW;
//     //     iop.size = 1.5e-3;
//     //     iop.color = glm::vec3(0.0, 0.0, 1.0);

//     //     iop.wireframe = false;

//     //     m_interaction_pts_ids.push_back(iop.id);
//     //     m_objects.push_back(std::make_shared<Point>());
//     //     m_objects.at(iop.id)->initialize_point(iop);
//     // }
// }

// // Remove interaction points
// void AnimationInterface::remove_interaction_points(void)
// {
//     // auto first = m_objects.end() - m_interaction_pts_ids.size();
//     // auto last = m_objects.end();
//     // m_objects.erase(first, last);
// }

// // Needle initialization
// void AnimationInterface::initialize_needle(void)
// {
//     if (m_needle_visual)
//     {
//         // Generate needle
//         generate_needle();

//         // Generate needle base frame
//         if (m_base_render) { generate_needle_base_frame(); }

//         // Generate handle
//         if (m_handle_render) { generate_handle(); }
//     }
//     else 
//     {
//         // Generate needle points
//         generate_needle_points();
//     }

// }

// // Generate needle points
// void AnimationInterface::generate_needle_points(void)
// {
//     // Get needle state
//     m_world_interface->get_needle_state(m_needle_pts_pos, m_needle_elements_rot);

//     // Set number of needle particles and elements
//     m_needle_np = m_needle_pts_pos.rows();
//     m_needle_nq = m_needle_elements_rot.size();

//     // Reshape neelde pts ids
//     m_needle_pts_ids.resize(m_needle_np);

//     // Reshape neelde elements ids
//     m_needle_elements_ids.resize(m_needle_nq);

//     // Generate quaternions
//     for (size_t i = 0; i < m_needle_nq; i++)
//     {
//         obj_type::ImportedSurfaceProperties iop;
//         iop.id = m_objects.size();
//         iop.pos = glm::vec3(0.0f, 0.0f, 0.0f);
//         iop.euler = glm::vec3(0.0f, 0.0f, 0.0f);
//         iop.shader_filename = m_simple_shader_path;
//         iop.mesh_filename = (m_current_path / "objects/frame/frame.obj").string();
//         iop.texture_filename = (m_current_path / "objects/frame/frame.png").string();
//         iop.scale = glm::vec3(0.005f, 0.005f, 0.005f);
//         iop.shininess = 64.0f;
//         iop.draw_type = GL_DYNAMIC_DRAW;
//         iop.wireframe = false;

//         m_needle_elements_ids.at(i) = iop.id;

//         m_objects.push_back(std::make_shared<ImportedSurface>());
//         m_objects.at(iop.id)->initialize_imported_surface(iop);
//     }

//     // Generate particles
//     for (size_t i = 0; i < m_needle_np; i++)
//     {
//         obj_type::PointProperties iop;
//         iop.id = m_objects.size();
//         iop.pos = glm::vec3(0.0f, 0.0f, 0.0f);
//         iop.size = 1.2e-3;
//         iop.shininess = 64.0f;
//         iop.draw_type = GL_DYNAMIC_DRAW;
//         iop.wireframe = false;
//         iop.color = glm::vec3(1.0, 0.0, 0.0);
//         m_needle_pts_ids.at(i) = iop.id;

//         m_objects.push_back(std::make_shared<Point>());
//         m_objects.at(iop.id)->initialize_point(iop);
//     }
// }

// // Update needle points
// void AnimationInterface::update_needle_points(void)
// {
//     // Get needle state
//     m_world_interface->get_needle_state(m_needle_pts_pos, m_needle_elements_rot);

//     // Initialize element positions
//     Eigen::MatrixXd elements_pos = Eigen::MatrixXd::Zero(m_needle_nq, 3);

//     // Loop through all points
//     for (size_t i = 0; i < m_needle_np; i++)
//     {
//         // Get position of point i
//         glm::vec3 pos_i = glm::vec3(m_needle_pts_pos(i, 0),
//             m_needle_pts_pos(i, 1), m_needle_pts_pos(i, 2));
        
//         m_objects.at(m_needle_pts_ids.at(i))->transform(pos_i,
//             glm::vec3(0.0, 0.0, 0.0));
    
//         // Generate element positions
//         if (i > 0)
//         {
//             elements_pos.row(i-1) = 0.5 * (m_needle_pts_pos.row(i-1) +
//                 m_needle_pts_pos.row(i));
//         }
//     }

//     // Update needle elements
//     for (size_t i = 0; i < m_needle_nq; i++)
//     {
//         // Get frame positions
//         Eigen::Matrix3d elements_rot = m_needle_elements_rot.at(i).toRotationMatrix();

//         // Set frame position
//         glm::vec3 frame_pos = glm::vec3(elements_pos(i, 0),
//             elements_pos(i, 1), elements_pos(i, 2));

//         // Set frame orientation
//         Eigen::Vector3d euler_angles =
//             dme::EulerRotations::rotation_to_euler(elements_rot);
        
//         // Set frame orientation
//         glm::vec3 frame_eul = glm::vec3(euler_angles(0), euler_angles(1), euler_angles(2));
        
//        m_objects.at(m_needle_elements_ids.at(i))->transform(frame_pos, frame_eul);
//     }
// }