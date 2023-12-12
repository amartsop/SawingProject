#include "../include/force_interface.h"


ForceInterface::ForceInterface(PhysicsParams* physics_params)
{
    // Store physics params
    m_physics_params = physics_params;

    // Initialize velocity sliding window
    m_vel_sliding_window = std::make_shared<utils::SlidingWindow>(10);
}

// Initialize tissue properties
void ForceInterface::initialize_tissue_properties(size_t tissue_particles_num,
    const Eigen::VectorXd& tissue_system_masses, const Eigen::MatrixXi&
    tissue_tet_elements)
{
    // Store number of particles
    m_tissue_particles_num = tissue_particles_num;

    // Store system masses 
    m_tissue_masses = tissue_system_masses;

    // Initialize force vector
    m_tissue_forces = Eigen::MatrixXd::Zero(m_tissue_particles_num, 3);

    // Initialize gravity 
    for (size_t i = 0; i < m_tissue_particles_num; i++)
    {
        m_tissue_forces(i, 2) = - m_tissue_masses(i) * m_physics_params->grav;
    }

    // Set tetrahedron mesh elements
    m_tissue_tet_elements = tissue_tet_elements;
}

// Initialize needle properties
void ForceInterface::initialize_needle_properties(size_t needle_particles_num,
    size_t needle_elements_num, const Eigen::VectorXd& needle_particle_masses,
    double needle_bevel_tip_angle)
{
    // Store number of needle particles
    m_needle_particles_num = needle_particles_num;

    // Store nubmer of needle elements
    m_needle_elements_num = needle_elements_num;

    // Initialize needle force vector
    m_needle_forces = Eigen::MatrixXd::Zero(m_needle_particles_num, 3);

    // Initialize needle torques vector
    m_needle_torques = Eigen::MatrixXd::Zero(m_needle_elements_num, 3);

    // Store needle's bevel tip angle 
    m_nbta = needle_bevel_tip_angle;
}

void ForceInterface::update(double real_time,
    const Eigen::MatrixXd& tissue_particles_pos,
    const Eigen::MatrixXd& needle_particles_pos, 
    const std::vector<Eigen::Quaterniond>& needle_elements_rot,
    const Eigen::MatrixXd& needle_particles_vel, 
    const std::vector<CollisionsInterface::NeedleCollisionInfo>&
    collision_info, const std::vector<int>& active_constraints_indices,
    utils::SystemState system_state)
{
    // Store tissue vertices
    m_tissue_tet_vertices = tissue_particles_pos;

    // Reset needle force vector
    m_needle_forces.setZero();

    // Reset tissue force vector
    m_tissue_forces.setZero();

    // Check if we are in the insertion phase
    if (system_state == utils::SystemState::insertion)
    {
        // Check whether we should activate cutting forces
        if (active_constraints_indices.size() > m_cfa_thresh)
        {
            // Calculate cutting forces
            calculate_cutting_forces(real_time, needle_particles_vel,
                needle_elements_rot, collision_info);
        }
    
        // Calculate friction forces
        calculate_friction_forces(real_time, needle_particles_vel,
            needle_elements_rot, collision_info, active_constraints_indices);
    }
}

// Calculate cutting forces
void ForceInterface::calculate_cutting_forces(double real_time,
    const Eigen::MatrixXd& needle_particles_vel, 
    const std::vector<Eigen::Quaterniond>& needle_elements_rot,
    const std::vector<CollisionsInterface::NeedleCollisionInfo>&
    collision_info)
{
    if (m_cutting_force_first_pass_flag)
    {
        // Set initial time
        m_cutting_force_init_time = real_time;
    
        // Update first pass flag
        m_cutting_force_first_pass_flag = false;
    }    

    // Execution time
    double exec_time = real_time - m_cutting_force_init_time;

    // Extract rotation matrix of needle tip element with respect to inertial
    Eigen::Matrix3d rot_fn_F = needle_elements_rot.back().toRotationMatrix();

    

    // Setup cutting force
    int last_element_idx = needle_particles_vel.rows() - 1;
    Eigen::Vector3d tip_vel_F = needle_particles_vel.row(last_element_idx).transpose();
    Eigen::Vector3d tip_vel_f = rot_fn_F.transpose() * tip_vel_F;
    double tip_vel_x = tip_vel_f(0);
    m_vel_sliding_window->push(tip_vel_x);
    
    double v = m_vel_sliding_window->get_mean();

    // m_fc_mag = 23.309 * m_vel_sliding_window->get_mean() + 0.4801;
    // m_fc_mag = 12551.8285 * pow(v, 3.) - 1338.1374 * pow(v, 2.) + 64.4271 * v + 0.1055;
    // m_fc_mag = 1.998401e-15 + 238.7656*v - 23236.8*pow(v,2) +
    //     1019543*pow(v,3) - 20079820*pow(v,4) + 146055800*pow(v,5);

    // Setup cutting force
    double fc = m_fc_mag * tanh(0.5 * exec_time);

    // Calculate cutting force with respect to the needle's tip frame
    Eigen::Vector3d fc_fn = Eigen::Vector3d(-fc * sin(m_nbta), 0.0, 
        -fc * cos(m_nbta));

    // Rotate cutting force to inertial frame
    Eigen::Vector3d fc_F = rot_fn_F * fc_fn;
    
    // Set cutting force to needle tip
    m_needle_forces.row(m_needle_particles_num-1) += fc_F.transpose();

    // Get tetrahedron index
    int tet_idx = collision_info.back().tet_element_id;
    
    // Get particle indices
    size_t idx0 = m_tissue_tet_elements(tet_idx, 0);
    size_t idx1 = m_tissue_tet_elements(tet_idx, 1);
    size_t idx2 = m_tissue_tet_elements(tet_idx, 2);
    size_t idx3 = m_tissue_tet_elements(tet_idx, 3);

    // Point force
    m_tissue_forces.row(idx0) += 0.25 * (-fc_F.transpose());
    m_tissue_forces.row(idx1) += 0.25 * (-fc_F.transpose());
    m_tissue_forces.row(idx2) += 0.25 * (-fc_F.transpose());
    m_tissue_forces.row(idx3) += 0.25 * (-fc_F.transpose());
}


// Calculate friction forces
void ForceInterface::calculate_friction_forces(double real_time,
    const Eigen::MatrixXd& needle_particles_vel, 
    const std::vector<Eigen::Quaterniond>& needle_elements_rot,
    const std::vector<CollisionsInterface::NeedleCollisionInfo>&
    collision_info, const std::vector<int>& active_constraints_indices)
{
    // Loop through active constraint indices
    for (size_t i = 1; i < active_constraints_indices.size(); i++)
    {
        // Active constaints indices
        int active_idx = active_constraints_indices.at(i);

        // Get tet id at active constraint
        int tet_idx = collision_info.at(active_idx).tet_element_id;

        // Define quaternions indices
        int quat_idx = active_idx - 1;

        // Correct quaternion indices
        if (quat_idx < 0) { quat_idx = 0; }

        // Get rotation matrix of element j wrt to F
        Eigen::Matrix3d rot_fj_F =
            needle_elements_rot.at(quat_idx).toRotationMatrix();
        
        // Get velocity of active particle 
        Eigen::Vector3d ropj_F_F_dot = needle_particles_vel.row(active_idx).transpose();

        // Get the velocity component in the insertion direction
        Eigen::Vector3d ropj_F_fj_dot = rot_fj_F.transpose() * ropj_F_F_dot;

        // Get velocity in the insertion direction
        double vel_insertion_dir = ropj_F_fj_dot(0);

        // Calculate friction in the insertion direction
        double friction_insertion_dir =
            m_friction_model.calculate_friction_force(vel_insertion_dir);

        // Calculate friction vector with respect to frame fj
        Eigen::Vector3d fr_fj = Eigen::Vector3d(friction_insertion_dir, 0.0, 0.0);

        // Transform friction to inertial frame
        Eigen::Vector3d fr_F = rot_fj_F * fr_fj;
    
        // Set needle forces
        m_needle_forces.row(active_idx) += fr_F.transpose();
    
        // Get particle indices
        size_t idx0 = m_tissue_tet_elements(tet_idx, 0);
        size_t idx1 = m_tissue_tet_elements(tet_idx, 1);
        size_t idx2 = m_tissue_tet_elements(tet_idx, 2);
        size_t idx3 = m_tissue_tet_elements(tet_idx, 3);

        // Point force
        m_tissue_forces.row(idx0) += 0.25 * (-fr_F.transpose());
        m_tissue_forces.row(idx1) += 0.25 * (-fr_F.transpose());
        m_tissue_forces.row(idx2) += 0.25 * (-fr_F.transpose());
        m_tissue_forces.row(idx3) += 0.25 * (-fr_F.transpose());
    }
}