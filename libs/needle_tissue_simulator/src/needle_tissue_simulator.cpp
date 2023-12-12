#include "../needle_tissue_simulator.h"

NeedleTissueSimulator::NeedleTissueSimulator(double ts,
    const std::string& current_filepath, const std::string& block_filename,
    const Eigen::Vector3d& roa_F_F, const Eigen::Vector3d& euler_f,
    const Eigen::Vector3d& rot_F_F, const Eigen::Vector3d& euler_ft)
{
    // Tissue model
    m_tissue_model = std::make_shared<TissueModel>(current_filepath,
        block_filename, rot_F_F, euler_ft, &m_physics_params);

    // Needle model
    int needle_elements_num = 30;
    m_needle_model = std::make_shared<NeedleModel>(roa_F_F, euler_f,
        &m_physics_params, needle_elements_num);

    // Needle tissue interface model
    m_needle_tissue_interface = std::make_shared<NeedleTissueInterface>(
        m_tissue_model, m_needle_model, &m_physics_params);

    // Force interface
    m_force_interface = std::make_shared<ForceInterface>(&m_physics_params);

    // Set simulation timestem
    m_dt = ts;
    
    // Initialize tissue tetrahedral mesh
    m_tissue_model->get_tetrahedral_mesh(m_tissue_tet_mesh);

    // Initialize needle state
    m_needle_model->get_state(m_needle_particles_pos, m_needle_elements_rotation);

    // Initialize tissue properties
    m_force_interface->initialize_tissue_properties(
        m_tissue_model->get_particles_number(),
        m_tissue_model->get_particles_masses(), m_tissue_tet_mesh.elements);

    // Set tissue insertion plane point indices
    m_force_interface->set_tissue_insertion_plane_point_indices(
        m_tissue_model->get_insertion_points_indices());

    // Initialize needle properties
    m_force_interface->initialize_needle_properties(
        m_needle_model->get_number_of_particles(),
        m_needle_model->get_number_of_elements(),
        m_needle_model->get_particles_masses(),
        m_needle_model->get_needle_bevel_tip_angle());

    // Initialize collision interface
    m_collisions_interface = std::make_shared<CollisionsInterface>(
        m_tissue_tet_mesh.elements, m_needle_particles_pos);
}

// Update xpbd
void NeedleTissueSimulator::update(double real_time, const Eigen::Vector3d&
    roa_F_F, const Eigen::Vector3d& eul_base)
{
    // Current substep 
    int current_substep = 0;    

    // Define substep interval
    double dts = m_dt / (double) m_substeps_num;

    // Update tissue tet mesh
    m_tissue_model->get_tetrahedral_mesh(m_tissue_tet_mesh);

    // Update needle state
    m_needle_model->get_state(m_needle_particles_pos, m_needle_elements_rotation);

    // Update needle velocities
    m_needle_model->get_velocities(m_needle_particles_vel,
        m_needle_elements_rot_vel);

    // Set desired needle base pose
    m_needle_model->set_base_pose(roa_F_F, eul_base);

    // Update collision interface
    Eigen::MatrixXd tissue_barycenters;     
    igl::barycenter(m_tissue_tet_mesh.vertices, m_tissue_tet_mesh.elements, 
        tissue_barycenters);

    // Update collision interface
    m_collisions_interface->update(m_tissue_tet_mesh.vertices,
        tissue_barycenters, m_needle_particles_pos);

    // Store needle collision info and active needle pts indices
    m_collisions_interface->get_needle_collision_info(m_needle_collision_info);
    m_collisions_interface->get_active_needle_points_ids(m_active_needle_points_indices);

    // Get current system state
    utils::SystemState current_system_state =
        m_needle_tissue_interface->get_current_system_state();

    // Update forces    
    m_force_interface->update(real_time, m_tissue_tet_mesh.vertices, 
        m_needle_particles_pos, m_needle_elements_rotation,
        m_needle_particles_vel, m_needle_collision_info,
        m_active_needle_points_indices, current_system_state);

    // Get external tissue forces    
    Eigen::MatrixXd external_tissue_forces;
    m_force_interface->get_external_tissue_forces(external_tissue_forces);

    // Get needle forces and torques
    Eigen::MatrixXd external_needle_forces, external_needle_torques;
    m_force_interface->get_external_needle_forces(external_needle_forces);
    m_force_interface->get_external_needle_torques(external_needle_torques);

    // Update needle tissue interface
    m_needle_tissue_interface->update(real_time, m_needle_collision_info,
        m_active_needle_points_indices, m_needle_elements_rotation,
        m_tissue_tet_mesh);

    // Perform substems
    while (current_substep < m_substeps_num)
    {
        // Predict needle state
        m_needle_model->predict_solution(real_time, dts, external_needle_forces, 
            external_needle_torques);

        // Predict tissue state
        m_tissue_model->predict_solution(real_time, dts, external_tissue_forces);
    
        // Iterate over constraints
        for (size_t i = 0; i < m_iterations_num; i++)
        {
            // Solve needle constraints
            m_needle_model->constrain_iteration(real_time, dts);

            // Solve tissue constraints
            m_tissue_model->constrain_iteration(real_time, dts);
    
            // Solve needle tissue interaction constrains
            m_needle_tissue_interface->constrain_iteration(real_time, dts);
        }

        // Update needle solution
        m_needle_model->update_solution(real_time, dts);
    
        // Update tissue solution
        m_tissue_model->update_solution(real_time, dts);

        

        // Update current substep
        current_substep += 1;
    }
}

// Get vertices
void NeedleTissueSimulator::get_tissue_vertices(Eigen::MatrixXd& tissue_vertices)
{
    // Get flattened vertices
    tissue_vertices =
        Mesh3D::get_flattened_tetrahedral_mesh(m_tissue_tet_mesh).vertices;
}

// Get faces
void NeedleTissueSimulator::get_tissue_faces(Eigen::MatrixXi& tissue_faces)
{
    // Get flattened faces
    tissue_faces =
        Mesh3D::get_flattened_tetrahedral_mesh(m_tissue_tet_mesh).faces;
}

// Get needle state
void NeedleTissueSimulator::get_needle_state(Eigen::MatrixXd&
    needle_particles_position, std::vector<Eigen::Quaterniond>&
    needle_elements_rotations)
{
    m_needle_model->get_state(needle_particles_position,
        needle_elements_rotations);
}

// Get boundary tissue points indices
void NeedleTissueSimulator::get_tissue_boundary_points(
    Eigen::MatrixXd& boundary_pts)
{
    boundary_pts = m_tissue_model->get_tissue_boundary_points();
}


// Get needle visual vertices
void NeedleTissueSimulator::get_needle_visual_vertices(
    const Eigen::Vector3d& rap_f_f_0,  Eigen::Vector3d& rop_F_F)
{
    m_needle_model->get_inertial_vertices(rap_f_f_0, rop_F_F);
}


// Get needle base state
void NeedleTissueSimulator::get_needle_base_state(Eigen::Vector3d& needle_pos,
    Eigen::Vector3d& needle_eul)
{
    m_needle_model->get_base_pose(needle_pos, needle_eul);
}


// Get needle generalized reaction force
void NeedleTissueSimulator::get_needle_generalized_reaction_force(
    Eigen::Vector3d& force, Eigen::Vector3d& moment)
{
    m_needle_model->get_needle_generalized_reaction_force(force, moment);
}

