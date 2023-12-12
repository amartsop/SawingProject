#include "../include/tissue_model.h"

TissueModel::TissueModel(const std::string& current_filepath,
    const std::string& block_filename, const Eigen::Vector3d& rot_F_F, 
    const Eigen::Vector3d& euler_ft, PhysicsParams* physics_params)
{
    // Define tissue mesh dimensions 
    m_tissue_mesh_dims = Eigen::Vector3d(physics_params->tissue_depth, 
        physics_params->tissue_width, physics_params->tissue_height);

    // Define tissue mesh
    m_tissue_mesh = std::make_shared<TissueMesh>(current_filepath, 
        rot_F_F, euler_ft, m_tissue_mesh_dims);

    // Get model mesh
    std::shared_ptr<Mesh3D> model_mesh = m_tissue_mesh->get_model_mesh();

    // Set initial mesh vertices
    m_initial_tet_mesh = model_mesh->get_tetrahedral_mesh();

    // Set current mesh vertices
    m_tet_mesh = model_mesh->get_tetrahedral_mesh();

    // Number of tetrahedrals
    m_tets_num = m_tet_mesh.elements.rows();

    // Current position
    m_pos = m_tet_mesh.vertices;

    // Current velocity 
    m_vel = Eigen::MatrixXd::Zero(m_pos.rows(), 3);

    // Get all mesh edges
    igl::edges(m_tet_mesh.elements, m_edges_ids);
    
    // Rest volume vector
    m_rest_volume = Eigen::VectorXd::Zero(m_tets_num, 1);

    // Edge lengths vector
    m_edge_lengths = Eigen::VectorXd::Zero(m_edges_ids.rows(), 1);

    // Set edge lengths
    for (size_t i = 0; i < m_edge_lengths.size(); i++)
    {
        // Get indices of edge vertices
        size_t idx0 = m_edges_ids(i, 0); size_t idx1 = m_edges_ids(i, 1);

        // Get points 
        Eigen::Vector3d p0 = m_tet_mesh.vertices.row(idx0).transpose();
        Eigen::Vector3d p1 = m_tet_mesh.vertices.row(idx1).transpose();
    
        // Calculate edge length
        m_edge_lengths(i) = (double) (p1 - p0).norm();
   }
    
    // Set boundary point indices
    m_boundary_points_indices = m_tissue_mesh->get_boundary_points_indices();
    
    // Set insertion point indices
    m_insertion_point_indices = m_tissue_mesh->get_insertion_plane_point_indices();
    
    // Set exterior normals
    m_tissue_exterior_normals = m_tissue_mesh->get_tissue_exterior_normals();
    
    // Set tissue boundary points
    m_tissue_boundary_pts = m_tissue_mesh->get_tissue_boundary_points();
    
    // Store physics params to member variable 
    m_physics_params_ptr = physics_params;    

    // Number of particles
    m_particles_num = m_tet_mesh.vertices.rows();

    // Set volume constraints num
    m_volume_constraints_num = m_tets_num;

    // Set edge constraints num
    m_edge_contstraints_num = m_edges_ids.rows();

    // Set boundart constraints num
    m_bc_constraints_num = m_boundary_points_indices.rows();
    
    // Inverse masses
    set_point_masses();

    // Initialize xpbd fem
    initialize_xpbd_fem();
    
    // Initialize lagrange multipliers
    initialize_lagrange_multipliers();
    
    // Initialize XPBD prediction
    m_x_tilde = Eigen::MatrixXd::Zero(m_particles_num, 3);
}

// Update 
void TissueModel::predict_solution(double real_time, double dt,
    const Eigen::MatrixXd& fext)
{
    // Initialize x_tilde        
    m_x_tilde.setZero();

    // Loop through points and predict positions
    for (size_t i = 0; i < m_particles_num; i++)
    {
        // // Calculcate predicted position
        // m_x_tilde.row(i) = m_pos.row(i) +
        //     m_vel.row(i) * dt + pow(dt, 2) * m_inv_masses(i) * fext.row(i);

        // Calculate temp predicted velocity 
        Eigen::Vector3d v_tilde = m_vel.row(i).transpose() +
            dt * m_inv_masses(i) * (fext.row(i).transpose() -
            m_trans_damping_matrix * m_vel.row(i).transpose());
        
        // Calculate temp predicted position 
        m_x_tilde.row(i) = m_pos.row(i) + dt * v_tilde.transpose();
    }

    // Reset Lagrange multipliers
    reset_lagrange_multipliers();
}

// Apply constraints 
void TissueModel::constrain_iteration(double real_time, double dt)
{
    // Realize boundary constraints
    XPBDConstraints::solve_point_boundary_constraints_single(dt, m_inv_masses,
        m_initial_tet_mesh.vertices, m_bc_compliance, m_boundary_points_indices,
        m_x_tilde, m_lambda_bc);

    // Solve Neo-Hookean hydrostatic constraints
    XPBDConstraints::solve_neo_hookean_hydrostatic_constraints(dt, 
        m_inv_masses,
        m_inv_dm_ref_shape_matrices,
        m_rest_volume,
        m_lame_mu,
        m_lame_lambda,
        m_tet_mesh.elements,
        m_x_tilde,
        m_lambda_nhh);

    // Solve Neo-Hookean deviatoric constraints
    XPBDConstraints::solve_neo_hookean_deviatoric_constraints(dt, 
        m_inv_masses,
        m_inv_dm_ref_shape_matrices,
        m_rest_volume,
        m_lame_mu,
        m_lame_lambda,
        m_tet_mesh.elements,
        m_x_tilde,
        m_lambda_nhd);
}

// Correct solution
void TissueModel::update_solution(double real_time, double dt)
{
    // Update velocities
    m_vel = (m_x_tilde -  m_pos) / dt;

    // Update position
    m_pos = m_x_tilde;

    // Update mesh
    m_tet_mesh.vertices = m_pos;
}

// Set point masses 
void TissueModel::set_point_masses(void)
{
    // Initialize masses
    m_masses = Eigen::VectorXd::Zero(m_particles_num, 1);

    // Initialize inverse masses
    m_inv_masses = Eigen::VectorXd::Zero(m_particles_num, 1);

    // Set inverse masses      
    for (size_t i = 0; i < m_tets_num; i++)
    {
        // Get indices of element i
        size_t idx_0 = m_tet_mesh.elements(i, 0);
        size_t idx_1 = m_tet_mesh.elements(i, 1);
        size_t idx_2 = m_tet_mesh.elements(i, 2);
        size_t idx_3 = m_tet_mesh.elements(i, 3);

        // Get vertices of element i
        Eigen::Vector3d p0 = m_tet_mesh.vertices.row(idx_0).transpose();
        Eigen::Vector3d p1 = m_tet_mesh.vertices.row(idx_1).transpose();
        Eigen::Vector3d p2 = m_tet_mesh.vertices.row(idx_2).transpose();
        Eigen::Vector3d p3 = m_tet_mesh.vertices.row(idx_3).transpose();

        // Calculate volume of tetrahedron i
        double tet_i_vol = XPBDConstraints::tetrahedron_volume(p0, p1, p2, p3);

        // Calculate mash of the tetrahedrin
        double tet_i_mass = m_physics_params_ptr->tissue_density * tet_i_vol;

        // Set rest volume
        m_rest_volume(i) = tet_i_vol;

        // Set masses
        m_masses(idx_0) += tet_i_mass / 4.0;
        m_masses(idx_1) += tet_i_mass / 4.0;
        m_masses(idx_2) += tet_i_mass / 4.0;
        m_masses(idx_3) += tet_i_mass / 4.0;
    }
    
    // Get inverse masses 
    for (size_t i = 0; i < m_inv_masses.size(); i++)
    {
        m_inv_masses(i) = 1.0 / m_masses(i);
    }
}

// Initialize xpbd fem
void TissueModel::initialize_xpbd_fem(void)
{
    // Initialize inverse reference shape matrices    
    m_inv_dm_ref_shape_matrices.resize(m_tets_num);

    // Initialize inverse matrices of elastic coefficients
    m_inv_elastic_coef_matrices.resize(m_tets_num);

    // Initialize Lame' constatins
    m_lame_mu = Eigen::VectorXd::Zero(m_tets_num);
    m_lame_lambda = Eigen::VectorXd::Zero(m_tets_num);

    // Loop through all the tetrahedron elements
    for (size_t i = 0; i < m_tets_num; i++)
    {
        // Get particle indices
        size_t idx1 = m_tet_mesh.elements(i, 0);
        size_t idx2 = m_tet_mesh.elements(i, 1);
        size_t idx3 = m_tet_mesh.elements(i, 2);
        size_t idx4 = m_tet_mesh.elements(i, 3);
                                    
        // Get coordinates of undeformed tetrahedron vertices
        Eigen::Vector3d x1_bar = m_tet_mesh.vertices.row(idx1).transpose();
        Eigen::Vector3d x2_bar = m_tet_mesh.vertices.row(idx2).transpose();
        Eigen::Vector3d x3_bar = m_tet_mesh.vertices.row(idx3).transpose();
        Eigen::Vector3d x4_bar = m_tet_mesh.vertices.row(idx4).transpose();

        // Calculate Dm matrix
        Eigen::Matrix3d dm_mat = Eigen::Matrix3d::Zero();
        dm_mat.col(0) = x1_bar - x4_bar;
        dm_mat.col(1) = x2_bar - x4_bar;
        dm_mat.col(2) = x3_bar - x4_bar;
    
        // Set inverse matrix
        m_inv_dm_ref_shape_matrices.at(i) = dm_mat.inverse();
    
        // Calculate matrix of elastic coefficients
        Eigen::Matrix<double, 6, 6> e_mat; e_mat.setZero();

        e_mat.block(0, 0, 3, 3) = m_physics_params_ptr->lambda *
            Eigen::Matrix3d::Ones();
        e_mat.block(0, 0, 3, 3) += (2.0 * m_physics_params_ptr->mu) *
            Eigen::Matrix3d::Identity();

        e_mat.block(3, 3, 3, 3) = (2.0 * m_physics_params_ptr->mu) *
            Eigen::Matrix3d::Identity();

        m_inv_elastic_coef_matrices.at(i) = e_mat.inverse();
    
        // Set Lame' constants for each element
        m_lame_lambda(i) = m_physics_params_ptr->lambda;
        m_lame_mu(i) = m_physics_params_ptr->mu;
    }

    // Set translations damping matrix
    m_trans_damping_matrix  = 1.e-1 * Eigen::Matrix3d::Identity();
}

// Initialize lagrange multipliers
void TissueModel::initialize_lagrange_multipliers(void)
{
    // Initialize lagrange multipliers for edge contraints
    m_lambda_edge = Eigen::VectorXd::Zero(m_edge_contstraints_num);

    // Initialize lagrange multipliers for volume constraints
    m_lambda_volume = Eigen::VectorXd::Zero(m_volume_constraints_num);

    // Initialize lagrange multipliers for boundary constraints
    m_lambda_bc = Eigen::VectorXd::Zero(m_bc_constraints_num);

    // Initialize Lagrange multipliers for Neo-Hookean hydrostatic model
    m_lambda_nhh = Eigen::VectorXd::Zero(m_tets_num);

    // Initialize Lagrange multipliers for Neo-Hookean deviatoric model
    m_lambda_nhd = Eigen::VectorXd::Zero(m_tets_num);

    // Initialize Lagrange multipliers for st venant - kirchhoff model
    m_lambda_svk = std::vector<Eigen::Matrix<double, 6, 1>>(m_tets_num,
        Eigen::Matrix<double, 6, 1>::Zero());
}

// Reset lagrange multipliers
void TissueModel::reset_lagrange_multipliers(void)
{
    // Initialize lagrange multipliers for edge contraints
    m_lambda_edge.setZero();

    // Initialize lagrange multipliers for volume constraints
    m_lambda_volume.setZero();

    // Initialize lagrange multipliers for boundary constraints
    m_lambda_bc.setZero();

    // Initialize Lagrange multipliers for st venant - kirchhoff model
    m_lambda_svk = std::vector<Eigen::Matrix<double, 6, 1>>(m_tets_num,
        Eigen::Matrix<double, 6, 1>::Zero());

    // Initialize Lagrange multipliers for Neo-Hookean hydrostatic model
    m_lambda_nhh.setZero();

    // Initialize Lagrange multipliers for Neo-Hookean deviatoric model
    m_lambda_nhd.setZero();
}