#include "../include/needle_tissue_interface.h"

NeedleTissueInterface::NeedleTissueInterface(const std::shared_ptr<TissueModel>&
    tissue_model, const std::shared_ptr<NeedleModel>& needle_model,
    PhysicsParams* physics_params)
{
    // Store tissue ptr    
    m_tissue_model = tissue_model;

    // Store needle ptr
    m_needle_model = needle_model;

    // Store phyiscs parameters ptr
    m_physics_params = physics_params;

    // Set number of needle particles
    m_np = m_needle_model->get_number_of_particles();

    // Get number of quaternions
    m_nq = m_needle_model->get_number_of_quaternions();

    // Get tissue inverse masses 
    m_tissue_inv_masses = m_tissue_model->get_particles_inverse_masses();

    // Store tissue elements
    Mesh3D::TetMesh tissue_tet_mesh;
    m_tissue_model->get_tetrahedral_mesh(tissue_tet_mesh);
    m_tissue_elements = tissue_tet_mesh.elements;

    // Get needle inverse masses 
    m_needle_inv_masses = m_needle_model->get_particles_inverse_masses();

    // Get needle inertial tensors
    m_needle_inertial_tensors = m_needle_model->get_elements_inertial_tensors();
    
    // Initialize collision array
    m_collision_array = Eigen::VectorXi::Constant(m_np, -1);

    // Collision array position
    m_collision_array_rab_f_f_0 = Eigen::MatrixXd::Zero(m_np, 3);

    // Collision array for rotations rot_f0_F
    m_collision_array_rot_f0_F = std::vector<Eigen::Matrix3d>(m_np,
        Eigen::Matrix3d::Zero());
    
    // L matrix
    m_l_mat = Eigen::Matrix3d::Identity();
    m_l_mat(0, 0) = 0.0;

    // Initialize contact normal (here assumed constant)
    m_contact_normal = Eigen::Vector3d(-1.0, 0.0, 0.0);

    // Initialize first contact natural coordinates
    m_first_contact_natural_coordinates.setZero();
}

// Update needle tissue interface
void NeedleTissueInterface::update(double real_time, const
    std::vector<CollisionsInterface::NeedleCollisionInfo>& collision_info,
    const std::vector<int>& active_constraints_indices,
    const std::vector<Eigen::Quaterniond>& needle_elements_rot,
    const Mesh3D::TetMesh& tet_mesh)
{
    // Update system state
    update_system_state();

    // Store active constraints indices
    m_active_constraints_indices = active_constraints_indices;

    // Store collision info
    m_collision_info = collision_info;

    // Update first contact handler
    update_first_contact_handler(tet_mesh);
    
    // State machine
    switch (m_current_state)
    {
    case utils::SystemState::stiffness:

        // Update stiffness phase
        update_stiffness_phase();

        break;

    case utils::SystemState::insertion:

        // Update stip phase    
        update_insertion_phase(real_time, tet_mesh, needle_elements_rot);
        break;
    
    default:
        break;
    }
}


// Constrain iteration
void NeedleTissueInterface::constrain_iteration(double real_time, double dt)
{
    // State machine
    switch (m_current_state)
    {
    case utils::SystemState::stiffness:

        // // If still in stiffness phase solve the contact constraints
        // XPBDConstraints::needle_tissue_tip_contact_constraint(dt, 
        //     m_tissue_inv_masses,
        //     m_needle_inv_masses,
        //     m_contact_compliance,
        //     m_active_constraints_indices,
        //     m_collision_info,
        //     m_first_contact_tet_id,
        //     m_first_contact_natural_coordinates,
        //     m_tissue_elements,
        //     m_contact_normal,
        //     *m_tissue_model->get_position_prediction_ptr(),
        //     *m_needle_model->get_position_prediction_ptr(),
        //     m_lambda_tissue_contact,
        //     m_lambda_needle_contact);
        
        break;

    case utils::SystemState::insertion:

        // Realize needle-tissue interaction constraints
        XPBDConstraints::needle_tissue_interaction_lateral_constraints_3d(dt, 
            m_tissue_inv_masses,
            m_needle_inv_masses,
            m_needle_inertial_tensors,
            m_nt_lateral_compliance,
            m_active_constraints_indices,
            m_collision_info,
            m_collision_array_rab_f_f_0,
            m_collision_array_rot_f0_F,
            m_tissue_elements,
            m_l_mat,
            *m_tissue_model->get_position_prediction_ptr(),
            *m_needle_model->get_position_prediction_ptr(),
            *m_needle_model->get_rotation_prediction_ptr(),
            m_lambda_lateral_nti);
        
        break;
    default:
        break;
    }
}


// Update first contact handler
void NeedleTissueInterface::update_first_contact_handler(const Mesh3D::TetMesh&
    tet_mesh)
{
    // Check if it is the first contact
    if (m_stiffness_phase_on && m_first_contact_flag)
    {
        // Active constaints indices
        int active_idx = m_active_constraints_indices.at(0);

        // Get tet id at active constraint
        m_first_contact_tet_id = m_collision_info.at(active_idx).tet_element_id;

        // Get particle indices
        size_t idx_a = tet_mesh.elements(m_first_contact_tet_id, 0);
        size_t idx_b = tet_mesh.elements(m_first_contact_tet_id, 1);
        size_t idx_c = tet_mesh.elements(m_first_contact_tet_id, 2);
        size_t idx_d = tet_mesh.elements(m_first_contact_tet_id, 3);

        // Get particles inertial positions
        Eigen::Vector3d xa = tet_mesh.vertices.row(idx_a).transpose();
        Eigen::Vector3d xb = tet_mesh.vertices.row(idx_b).transpose();
        Eigen::Vector3d xc = tet_mesh.vertices.row(idx_c).transpose();
        Eigen::Vector3d xd = tet_mesh.vertices.row(idx_d).transpose();

        // Get position of contact point
        Eigen::Vector3d xq = m_collision_info.at(active_idx).point_inertial_position;

        // Extract the natural coordinates on the contact point
        m_first_contact_natural_coordinates =
            XPBDConstraints::extract_linear_tet_natural_coordinates(xq, xa, xb,
            xc, xd);
    
        // Update first contact flag
        m_first_contact_flag = false;
    }
}

// Update stiffness phase
void NeedleTissueInterface::update_stiffness_phase(void)
{
    // Reset Lagrange multiplier for needle/tissue contact
    m_lambda_tissue_contact = 0.0; m_lambda_needle_contact = 0.0;
}


// Update system state
void NeedleTissueInterface::update_system_state(void)
{
    // Get reaction force
    Eigen::Vector3d force, moments;
    m_needle_model->get_needle_generalized_reaction_force(force, moments);
    bool stiffness_force_flag = (force(0) < m_stiffness_force_thresh);
    
    /* Check if we should solve contact (stiffness phase). The way it is permorfed
    avoids getting in and out of stiffness phase due to the vibrations on the
    stiffness force flag */
    if (m_active_constraints_indices.size() == 1)
    {
        if (stiffness_force_flag && m_stiffness_first_pass)
        {
            m_stiffness_phase_on = true;
        }
        else 
        {
            m_stiffness_phase_on = false;

            // Deactiveate first pass flag
            m_stiffness_first_pass = false;
        }
    }

    // Needle outside the tissue
    if (m_active_constraints_indices.size() < 1)
    {
        m_current_state = utils::SystemState::outside;
    }
    // else if(m_stiffness_phase_on)
    // {
    //     m_current_state = utils::SystemState::stiffness;
    // }
    else 
    {
        m_current_state = utils::SystemState::insertion;
    }

    
}

// Update insertion phase
void NeedleTissueInterface::update_insertion_phase(double real_time,
    const Mesh3D::TetMesh& tet_mesh, const std::vector<Eigen::Quaterniond>&
    needle_elements_rot)
{
    // Update lateral constraints
    update_lateral_constraints(real_time, tet_mesh, needle_elements_rot);
}

// Update lateral constraints
void NeedleTissueInterface::update_lateral_constraints(double real_time,
    const Mesh3D::TetMesh& tet_mesh,
    const std::vector<Eigen::Quaterniond>& needle_elements_rot)
{
    // Check if the needle particle has moved to the next tetrahedron
    for (size_t i = 0; i < m_active_constraints_indices.size(); i++)
    {
        // Active constaints indices
        int active_idx = m_active_constraints_indices.at(i);
        
        // Get tet id at active constraint
        int tet_id = m_collision_info.at(active_idx).tet_element_id;

        // Check for condition and update collision array
        if ( tet_id != m_collision_array(active_idx))
        {
            // Define quaternions indices
            int quat_idx = active_idx - 1;

            // Correct quaternion indices
            if (quat_idx < 0) { quat_idx = 0; }
            
            // Get rotation matrix of element j wrt to F
            Eigen::Matrix3d rot_fj_F =
                needle_elements_rot.at(quat_idx).toRotationMatrix();
            
            // Vector roa_f_f_0
            Eigen::Vector3d roa_F_F_0 =
                m_collision_info.at(active_idx).point_inertial_position;

            // Get particle indices
            size_t idx0 = tet_mesh.elements(tet_id, 0);
            size_t idx1 = tet_mesh.elements(tet_id, 1);
            size_t idx2 = tet_mesh.elements(tet_id, 2);
            size_t idx3 = tet_mesh.elements(tet_id, 3);

            // Get particles inertial positions
            Eigen::Vector3d xa_0 = tet_mesh.vertices.row(idx0).transpose();
            Eigen::Vector3d xb_0 = tet_mesh.vertices.row(idx1).transpose();
            Eigen::Vector3d xc_0 = tet_mesh.vertices.row(idx2).transpose();
            Eigen::Vector3d xd_0 = tet_mesh.vertices.row(idx3).transpose();

            // Vector rob_f_f_0
            Eigen::Vector3d rob_F_F_0 = 0.25 * (xa_0 + xb_0 + xc_0 + xd_0);

            // Vector rab_f_f_0
            Eigen::Vector3d rab_f_f_0 = rot_fj_F.transpose() * (rob_F_F_0 -
                roa_F_F_0);

            // Update collision array
            m_collision_array(active_idx) = tet_id;

            // Update initial contact position
            m_collision_array_rab_f_f_0.row(active_idx) = rab_f_f_0.transpose();

            // Update initial contact rotation
            m_collision_array_rot_f0_F.at(active_idx) = rot_fj_F;
        }
    }
    
    // Lagrange multipliers for lateral needle/tissue interaction
    m_lambda_lateral_nti = std::vector<Eigen::Vector3d>(
        m_active_constraints_indices.size(), Eigen::Vector3d::Zero());
}