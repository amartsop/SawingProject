#include "../include/needle_model.h"

NeedleModel::NeedleModel(const Eigen::Vector3d& roa_F_F,
    const Eigen::Vector3d& euler_f, PhysicsParams* physics_params,
    int needle_elements)
{
    // Physics params
    m_physics_params = physics_params;

    // Set number of elements    
    m_ne = needle_elements;

    // Set zero quat 
    m_zero_quat.coeffs().setZero();   

    // Elements length    
    m_element_length = m_physics_params->needle_length / m_ne;
    
    // Set number of particles and number of quaternions
    m_np = m_ne + 1; m_nq = m_ne;

    // Initialize state
    m_pos = Eigen::MatrixXd::Zero(m_np, 3);
    m_vel = Eigen::MatrixXd::Zero(m_np, 3);
    m_vel_prev = Eigen::MatrixXd::Zero(m_np, 3);

    // Initialize quaternions and rotation velocities
    m_w = Eigen::MatrixXd::Zero(m_nq, 3);
    m_quat = std::vector<Eigen::Quaterniond>(m_nq, Eigen::Quaterniond::Identity());

    // Get initial rotation matrix
    Eigen::Quaterniond quat0 = dme::EulerRotations::euler_to_quaternions(
        euler_f(0), euler_f(1), euler_f(2));
    Eigen::Matrix3d rot_f_F_0 = quat0.toRotationMatrix();
    
    // Initialize state 
    for (size_t i = 0; i < m_np; i++)
    {
        // Calculate relative position vector
        Eigen::Vector3d rap_f_f =  Eigen::Vector3d((double)i*m_element_length, 0.0, 0.0);       

        // Calculate absolute position vector
        Eigen::Vector3d rop_F_F = roa_F_F + rot_f_F_0 * rap_f_f;

        m_pos.row(i) = rop_F_F.transpose();
    }
    
    // Set initial position
    m_pos0 = m_pos;

    // Initialize inertial properties
    define_inertial_properties();

    // Initialize cosserat constraints
    initialize_cosserat_constraints();

    // Initialize boundary conditions
    initialize_boundary_conditions(roa_F_F, quat0);

    // Initialize XPBD prediction for particles
    m_x_tilde = Eigen::MatrixXd::Zero(m_np, 3);

    // Initialize prediction for elements rotations
    m_q_tilde = std::vector<Eigen::Quaterniond>(m_nq,
        Eigen::Quaterniond::Identity());

    // Set Cosserat state
    m_cosserat_state.particles_pos.resize(m_np);
    m_cosserat_state.quaternions_rot.resize(m_nq);
    m_cosserat_state.quaternions_pos.resize(m_nq);

    // Set initial cosserat state
    update_cosserat_state();
    m_init_cosserat_state = m_cosserat_state;

    // Calculative length per element in the undeflected configuration
    for (size_t i = 0; i < m_ne; i++)
    {
        m_cel.push_back(m_element_length * i);
    }
}

// Predict solution
void NeedleModel::predict_solution(double real_time, double dt,
    const Eigen::MatrixXd& fext, const Eigen::MatrixXd& text)
{
    // Initialize XPBD prediction for particles
    m_x_tilde.setZero();

    // Initialize prediction for elements rotations
    m_q_tilde = std::vector<Eigen::Quaterniond>(m_nq,
        Eigen::Quaterniond::Identity());

    // Loop through points and predict positions
    for (size_t i = 0; i < m_np; i++)
    {
        // Calculate temp predicted velocity 
        Eigen::Vector3d v_tilde = m_vel.row(i).transpose() +
            dt * m_inv_masses(i) * (fext.row(i).transpose() -
            m_trans_damping_matrix * m_vel.row(i).transpose());
        
        // Calculate temp predicted position 
        m_x_tilde.row(i) = m_pos.row(i) + dt * v_tilde.transpose();
    }   

    // Loop through elements and predict rotations
    for (size_t i = 0; i < m_nq; i++)
    {
        // Calculate temp predicted rotational velocity
        Eigen::Vector3d w_current = m_w.row(i).transpose();
        Eigen::Vector3d w_tilde = w_current +
            dt * m_inv_inertia_tensors.at(i) * (text.row(i).transpose()
            - m_bending_damping_matrix * w_current -
            dme::S(w_current) * (m_inertia_tensors.at(i) * w_current));

        // Convert w_tilde to quaternion
        Eigen::Quaterniond w_tilde_quat = Eigen::Quaterniond(0, 
            w_tilde(0), w_tilde(1), w_tilde(2));
        
        // Calculate the predicted rotation
        Eigen::Quaterniond q_tilde_corr = m_quat.at(i) * w_tilde_quat;
        q_tilde_corr.coeffs() *= 0.5 * dt;
        
        m_q_tilde.at(i).coeffs() = m_quat.at(i).coeffs() + q_tilde_corr.coeffs();
    
        // Normalize prediction
        m_q_tilde.at(i).normalize();
    }

    // Initialize Langrange multipliers for stretch/shear
    m_lambda_stretch_shear = std::vector<Eigen::Vector3d>(m_pcp_num,
        Eigen::Vector3d::Zero());

    // Initialize Langrange multipliers for bendling/torsion
    m_lambda_bending_torsion = std::vector<Eigen::Vector3d>(m_ecp_num,
        Eigen::Vector3d::Zero());

    // Initialize Langrange multipliers for position boundary
    m_lambda_position_bound = std::vector<Eigen::Vector3d>(
        m_boundary_points_indices.rows(), Eigen::Vector3d::Zero());
    
    // Initialize Lagrange multipliers for quaternion boundary
    m_lambda_quat_bound = std::vector<Eigen::Vector3d>(
        m_boundary_quat_indices.rows(), Eigen::Vector3d::Zero());
}

// Apply constraints
void NeedleModel::constrain_iteration(double real_time, double dt)
{
    // Realize cosserat stretch/shear constraints
    XPBDConstraints::cosserat_stretch_shear_constraints(dt, m_inv_masses,
        m_inertia_tensors, m_element_length,
        m_stretch_compliance_matrix, m_pcp, m_x_tilde,
        m_q_tilde, m_lambda_stretch_shear);

    // Realize cosserat bending/torsion constraints
    XPBDConstraints::cosserat_bend_twist_constraints(dt, m_inertia_tensors,
        m_element_length, m_bend_compliance_matrix, m_rest_darboux_vector,
        m_ecp, m_q_tilde, m_lambda_bending_torsion);

    // Point boundary constraint
    XPBDConstraints::solve_point_boundary_constraints(dt, m_inv_masses, 
        m_boundary_points_compliance, m_desired_points_positions,
        m_boundary_points_indices, m_x_tilde, m_lambda_position_bound);

    // Point boundary constraint
    XPBDConstraints::solve_quaternion_boundary_constraints(dt,
        m_inv_inertia_tensors, m_desired_quaternions, 
        m_boundary_quat_compliance, m_boundary_quat_indices,
        m_q_tilde, m_lambda_quat_bound);
}

// Correct solutions
void NeedleModel::update_solution(double real_time, double dt)
{
    // Update velocities
    m_vel = (m_x_tilde -  m_pos) / dt;

    // Update position
    m_pos = m_x_tilde;
    
    // Update rotations
    for (size_t i = 0; i < m_ne; i++)
    {
        // Normalize prediction
        m_q_tilde.at(i).normalize();

        // Update rotation
        Eigen::Quaterniond w_update; w_update.coeffs().setZero();
        w_update = m_quat.at(i).conjugate() * m_q_tilde.at(i);
        w_update.coeffs() *= (2.0 / dt);
        m_w.row(i) = w_update.vec().transpose();
    }

    // Update rotations
    m_quat = m_q_tilde;

    // Update visual state
    update_cosserat_state();

    // Calculate reaction forces
    calculate_reaction_forces(dt);

    // Store previous velocity
    m_vel_prev = m_vel;
}

// Define inertial properties
void NeedleModel::define_inertial_properties(void)
{
    /****************** Particles Masses *********************/
    // Initialize particles masses 
    m_masses = Eigen::VectorXd::Zero(m_np);
    m_inv_masses = Eigen::VectorXd::Zero(m_np);

    // Cross-sectional element area
    double element_area = M_PI * pow(m_physics_params->needle_radius , 2.0);   

    // Calculate needle total mass     
    double needle_mass = m_physics_params->needle_density * element_area *
        m_physics_params->needle_length;
    double mass_per_particle = needle_mass / (double) m_np;

    // Distribute masses to particles (lumbed mass model)
    mass_per_particle = 0.01;
    m_needle_mass = mass_per_particle * m_np;
    double inv_mass_per_particle = 1.0 / mass_per_particle;

    // Set particle masses
    m_masses = mass_per_particle * Eigen::VectorXd::Ones(m_np, 1);
    m_inv_masses = inv_mass_per_particle * Eigen::VectorXd::Ones(m_np, 1);

    /****************** Elements' inertias *********************/
    // Initialize elements' inertias
    Eigen::Matrix3d inertia_per_element = Eigen::Matrix3d::Zero();
    inertia_per_element(0, 0) = 0.5 * mass_per_particle *
        pow(m_physics_params->needle_radius, 2.0);
    inertia_per_element(1, 1) = (1.0 / 12.0) * mass_per_particle *
        ( 3.0 * pow(m_physics_params->needle_radius, 2.0) + pow(m_element_length, 2.0));
    inertia_per_element(2, 2) = inertia_per_element(1, 1);

    // Inverse inertia per element
    Eigen::Matrix3d inv_inertia_per_element = inertia_per_element.inverse();
    
    // Set elements inertials
    m_inertia_tensors = std::vector<Eigen::Matrix3d>(m_nq, inertia_per_element);

    // Set elements inverse inertial
    m_inv_inertia_tensors = std::vector<Eigen::Matrix3d>(m_nq,
        inv_inertia_per_element);
}

// Initialize cosserat contraints
void NeedleModel::initialize_cosserat_constraints(void)
{
    /***************** Particles connection pairs *********************/
    // Define number of particles connection pairs
    m_pcp_num = m_ne;
    
    // Define particles connection pairs
    m_pcp = Eigen::MatrixXd::Zero(m_pcp_num, 2);

    for (size_t i = 0; i < m_pcp_num ; i++)
    {
        // m_pcp(m_pbio(i), 0) = i; m_pcp(m_pbio(i), 1) = i+1;
        m_pcp(i, 0) = i; m_pcp(i, 1) = i+1;
    }

    /***************** Elements connection pairs *********************/
    // Define number of elements connection pairs
    m_ecp_num = m_ne - 1; 

    // Define element connection pairs
    m_ecp = Eigen::MatrixXd::Zero(m_ecp_num, 2);

    for (size_t i = 0; i < m_ecp_num; i++)
    {
        m_ecp(i, 0) = i; m_ecp(i, 1) = i+1;
    }

    // Initialize rest darboux vector
    m_rest_darboux_vector = XPBDConstraints::calculate_darboux_vector(
        Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(),
        m_element_length);

    // Generate cosserat compliance matrices
    generate_cosserat_compliance_matrices();
}

// Generate bending stiffness matrices
void NeedleModel::generate_cosserat_compliance_matrices(void)
{
    // Cross-sectional area
    double element_area = M_PI * pow(m_physics_params->needle_radius, 2.0);

    // Element moments of area
    double element_iyy = M_PI * pow(m_physics_params->needle_radius, 4.0) / 4.0;
    double element_izz = M_PI * pow(m_physics_params->needle_radius, 4.0) / 4.0;
    double element_ixx = element_iyy + element_izz;

    // Bending/torsion matrix
    Eigen::Matrix3d k_mat = Eigen::Matrix3d::Zero(3, 3);
    k_mat(0, 0) = m_physics_params->needle_shear_modulus * element_ixx;
    k_mat(1, 1) = m_physics_params->needle_young_modulus * element_iyy;
    k_mat(2, 2) = m_physics_params->needle_young_modulus * element_izz;
    
    // Build matrix
    m_bend_compliance_matrix = k_mat.inverse();

    // Set translations damping matrix
    m_trans_damping_matrix  = 1.e-2 * Eigen::Matrix3d::Identity();
    m_trans_damping_matrix(0, 0) = 1.0;

    // Set bending damping matrix
    m_bending_damping_matrix  = 1.e-3 * Eigen::Matrix3d::Identity();
    m_bending_damping_matrix(0, 0) = 0.0;

    // Set stretch compliance matrices
    m_stretch_compliance_matrix  = 1.e-100 * Eigen::Matrix3d::Identity();
}

// Initialize boundary conditions 
void NeedleModel::initialize_boundary_conditions(
    const Eigen::Vector3d& roa_F_F_0, const Eigen::Quaterniond& quat0)
{
    // Number of driving points
    int dp_num = 1;
    
    /************** Position Constraints **********/
    // Set initial base pose    
    m_roa_F_F = roa_F_F_0; m_quat_base = quat0;

    // Setup driving point
    m_desired_points_positions = Eigen::MatrixXd::Zero(m_np, 3);
    
    // Set position of driving point
    m_desired_points_positions.row(0) = roa_F_F_0.transpose();

    // Initialize Boundary compliance 
    m_boundary_points_compliance = 1.e-100 * Eigen::Matrix3d::Identity();

    // Initialize boundary points indices 
    m_boundary_points_indices = Eigen::VectorXi::Zero(dp_num, 1);

    /************** Rotation Constraints **********/
    // Initialize desired quaternions
    m_desired_quaternions = std::vector<Eigen::Quaterniond>(m_nq,
        Eigen::Quaterniond::Identity());

    m_desired_quaternions.at(0) = quat0;

    // Initialize boundary quaternions indices (only first quaternion with index 0)
    m_boundary_quat_indices = Eigen::VectorXi::Zero(dp_num, 1);

    // Boundary compliance for quaternion
    m_boundary_quat_compliance = 0.9e-19 * Eigen::Matrix3d::Identity();
}

// Set base pose
void NeedleModel::set_base_pose(const Eigen::Vector3d& roa_F_F,
    const Eigen::Vector3d& euler_f)
{
    // Store base pose 
    m_roa_F_F = roa_F_F;
    
    // Set boundary conditions
    m_desired_points_positions.row(0) = roa_F_F.transpose();
    
    // Store base quaternion
    m_quat_base = dme::EulerRotations::euler_to_quaternions(euler_f(0),
        euler_f(1), euler_f(2));

    // Set boundary conditions
    m_desired_quaternions.at(0) = m_quat_base;
}

// Update Cosserat state
void NeedleModel::update_cosserat_state(void)
{
    // Set base position
    m_roa_F_F = m_x_tilde.row(0).transpose();

    // Set base orientation
    m_quat_base = m_quat.at(0);

    // Update positions
    for (size_t i = 0; i < m_np; i++)    
    {
        m_cosserat_state.particles_pos.at(i) = m_pos.row(i).transpose();
    }

    // Update rotations
    for (size_t i = 0; i < m_nq; i++)
    {
        m_cosserat_state.quaternions_rot.at(i) = m_quat.at(i).matrix();
        m_cosserat_state.quaternions_pos.at(i) = 0.5 *
            (m_pos.row(i+1).transpose() + m_pos.row(i).transpose());
    }
}

// Get initial state
void NeedleModel::get_initial_state(int elem_idx, Eigen::Vector3d& roa_F_F_0,
    Eigen::Matrix3d& rot_f_F_0, Eigen::Vector3d& raaj_f_f_0, 
    Eigen::Matrix3d& rot_fj_f_0)
{
    // Set initial roa_F_F
    roa_F_F_0 = m_init_cosserat_state.particles_pos.at(0);

    // Set initial rot_f_F_0
    rot_f_F_0 = m_init_cosserat_state.quaternions_rot.at(0);

    // Set the initial roaj_F_F
    Eigen::Vector3d roaj_F_F = m_init_cosserat_state.quaternions_pos.at(elem_idx);

    // Set the initial raaj_f_f_0
    raaj_f_f_0 = rot_f_F_0.transpose() * (roaj_F_F - roa_F_F_0);
    
    // Set initial rot_fj_f_0 (assume undeflected)
    rot_fj_f_0 = Eigen::Matrix3d::Identity();
}

// Get needle visual vertices
void NeedleModel::get_inertial_vertices(const Eigen::Vector3d&
    rap_f_f_0, Eigen::Vector3d& rop_F_F)
{
    // Find the idx of the element the rap_f_f belongs to    
    int elem_idx = search_sorted_double(m_cel, rap_f_f_0(0));

    // Define xj (the x position of raaj_f_f)
    int quat_idx = 0;
    if (elem_idx == 0) { quat_idx = 0; }
    else { quat_idx = elem_idx -1; }
    
    // Get initial state for rap_f_f
    Eigen::Vector3d roa_F_F_0;
    Eigen::Matrix3d rot_f_F_0;
    Eigen::Vector3d raaj_f_f_0; 
    Eigen::Matrix3d rot_fj_f_0;
    get_initial_state(quat_idx, roa_F_F_0, rot_f_F_0, raaj_f_f_0, rot_fj_f_0);

    // Get constant rajpj_fj_fj
    Eigen::Vector3d rajpj_fj_fj = rot_fj_f_0.transpose() * (rap_f_f_0 -
        raaj_f_f_0);
    
    // Get roaj_F_F
    Eigen::Vector3d roaj_F_F = m_cosserat_state.quaternions_pos.at(quat_idx);

    // Get rot_fj_F
    Eigen::Matrix3d rot_fj_F = m_cosserat_state.quaternions_rot.at(quat_idx);
    
    // Get ropj_F_F
    rop_F_F = roaj_F_F + rot_fj_F * rajpj_fj_fj;    
}

// Search sorted algorithm
int NeedleModel::search_sorted_double(const std::vector<double>& array,
    double element)
{
    // Define array length
    int n = array.size();

    // Lower and upper bounds
    int start = 0;
    int end = n - 1;

    // Traverse the search space
    while (start <= end) {

        int mid = (start + end) / 2;

        // If element is found
        if (array.at(mid) == element)
            return mid;
        else if (array.at(mid) < element)
            start = mid + 1;
        else
            end = mid - 1;
    }

    // Return insert position
    return end + 1;
}

// Calculate reaction forces 
void NeedleModel::calculate_reaction_forces(double dt)
{
    // Calculate fa_F    
    m_reaction_force = m_reaction_force_coeff *
        m_lambda_position_bound.at(0);

    // Get euler angles 
    Eigen::Vector3d eul0 = dme::EulerRotations::quaternions_to_euler(m_quat.at(0));

    // Calculate g matrix 
    Eigen::Matrix3d g_mat = dme::EulerRotations::G(eul0);
    
    // Get reaction moment at A
    Eigen::Vector3d ma = m_reaction_moment_coeff * m_lambda_quat_bound.at(0);
    m_reaction_moment = (g_mat.transpose()).colPivHouseholderQr().solve(ma);
    m_reaction_force(1) *= -1;
}