#include "../include/extended_pbd.h"

st::ExtendedPBD::ExtendedPBD(const st::Mesh3D::TetMesh& tet_mesh,
    PhysicsParams* physics_params, const Eigen::VectorXi& boundary_point_indices,
    const Eigen::VectorXi& insertion_point_indices,
    const Eigen::MatrixXd& exterior_normals)
{
//     // Store physics params to member variable 
//     m_physics_params_ptr = physics_params;    

//     // Number of particles
//     m_particles_num = tet_mesh.vertices.rows();

//     // Set initial mesh vertices
//     m_initial_tet_mesh = tet_mesh;

//     // Set current mesh vertices
//     m_tet_mesh = tet_mesh;

//     // Number of tetrahedrals
//     m_tets_num = m_tet_mesh.elements.rows();

//     // Current position
//     m_pos = m_tet_mesh.vertices;

//     // Current velocity 
//     m_vel = Eigen::MatrixXd::Zero(m_pos.rows(), 3);

//     // Get all mesh edges
//     igl::edges(m_tet_mesh.elements, m_edges_ids);
    
//     // Rest volume vector
//     m_rest_volume = Eigen::VectorXd::Zero(m_tets_num, 1);

//     // Edge lengths vector
//     m_edge_lengths = Eigen::VectorXd::Zero(m_edges_ids.rows(), 1);

//     // Set edge lengths
//     for (size_t i = 0; i < m_edge_lengths.size(); i++)
//     {
//         // Get indices of edge vertices
//         size_t idx0 = m_edges_ids(i, 0);
//         size_t idx1 = m_edges_ids(i, 1);

//         // Get points 
//         Eigen::Vector3d p0 = m_tet_mesh.vertices.row(idx0).transpose();
//         Eigen::Vector3d p1 = m_tet_mesh.vertices.row(idx1).transpose();
    
//         // Calculate edge length
//         m_edge_lengths(i) = (double) (p1 - p0).norm();
//    }
    
//     // Set boundary point indices
//     m_boundary_points_indices = boundary_point_indices;
    
//     // Set insertion point indices
//     m_insertion_point_indices = insertion_point_indices;
    
//     // Set exterior normals
//     m_tissue_exterior_normals = exterior_normals;
    
//     // Inverse masses
//     set_point_masses();
    
//     // Make force interface handler
//     m_force_ptr = std::make_shared<st::ForceInterface>(m_particles_num,
//         physics_params, m_masses, m_tet_mesh.elements);

//     m_force_ptr->set_insertion_plane_point_indices(insertion_point_indices);

//     // Set volume constraints num
//     m_volume_constraints_num = m_tets_num;

//     // Set edge constraints num
//     m_edge_contstraints_num = m_edges_ids.rows();

//     // Set boundart constraints num
//     m_bc_constraints_num = m_boundary_points_indices.rows();

//     ///////////////////////////////////////////////////////////////////

//     // Initialize collision array
//     m_collision_array = Eigen::VectorXi::Constant(30, -1);

//     // Collision array position
//     m_collision_array_rab_f_f_0 = Eigen::MatrixXd::Zero(30, 3);

//     // Collision array rotation
//     m_collision_array_rotation = std::vector<Eigen::Matrix3d>(30,
//         Eigen::Matrix3d::Identity());

//     // L matrix
//     m_l_mat = Eigen::Matrix3d::Identity(); m_l_mat(0, 0) = 0.0;

//     ///////////////////////////////////////////////////////////////////
}

// Update xpbd
void st::ExtendedPBD::update(double real_time)
{
    // // Initialize x_tilde        
    // Eigen::MatrixXd x_tilde = Eigen::MatrixXd::Zero(m_particles_num, 3);

    // //Update forces
    // Eigen::MatrixXd fext = m_force_ptr->get_force_vector(m_pos, real_time);

    // // Loop through points and predict positions
    // for (size_t i = 0; i < m_particles_num; i++)
    // {
    //     // Calculcate predicted position
    //     x_tilde.row(i) = m_pos.row(i) +
    //         m_vel.row(i) * m_dt + pow(m_dt, 2) * m_inv_masses(i) * fext.row(i);
    // }

    // // Initialize lagrange multipliers for edge contraints
    // Eigen::VectorXd lambda_edge = Eigen::VectorXd::Zero(m_edge_contstraints_num);

    // // Initialize lagrange multipliers for volume constraints
    // Eigen::VectorXd lambda_volume = Eigen::VectorXd::Zero(m_volume_constraints_num);

    // // Initialize lagrange multipliers for boundary constraints
    // Eigen::VectorXd lambda_bc = Eigen::VectorXd::Zero(m_bc_constraints_num);

    // // Initialize lagrange multipliers for needle tissue interaction
    // std::vector<int> active_constraints_indices;
    // for (size_t i = 0; i < collision_info.size(); i++)    
    // {
    //     if (collision_info.at(i).collision_status)
    //     {
    //         active_constraints_indices.push_back(i);
    //     }
    // }

    // // Update collision arrays        
    // update_nt_collision_array(collision_info, active_constraints_indices);

    // Eigen::VectorXd lambda_nt = Eigen::VectorXd::Zero(active_constraints_indices.size());

    // for (size_t i = 0; i < m_iterations_num; i++)
    // {
    //     // Realize edge constraints
    //     solve_edge_constraints(m_dt, x_tilde, lambda_edge);

    //     // Realize volume constraints
    //     solve_volume_constraints(m_dt, x_tilde, lambda_volume);
    
    //     // Realize boundary constraints
    //     solve_boundary_constraints(real_time, m_dt, x_tilde, lambda_bc);
    
    //     // Solve needle tissue interaction constraints
    //     solve_needle_tissue_interation_constraints(collision_info,
    //         active_constraints_indices, m_dt, x_tilde, lambda_nt);
    // }

    // // Update velocities
    // m_vel = (x_tilde -  m_pos) / m_dt;

    // // Update position
    // m_pos = x_tilde;

    // // Update mesh
    // m_tet_mesh.vertices = m_pos;
}

// // Calculate tetrahedron volume
// double st::ExtendedPBD::tetrahedron_volume(const Eigen::Vector3d& p0,
//     const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
//     const Eigen::Vector3d& p3)
// {
//     double vol = (1. / 6.) * (p1 - p0).cross(p2 - p0).dot(p3 - p0);
//     return std::abs(vol);
// }

// // Solve edge contraints
// void st::ExtendedPBD::solve_edge_constraints(double dts, Eigen::MatrixXd& x_tilde, 
//     Eigen::VectorXd& lambda_edge)
// {
//     // Calculate alpha (same for all contraints of the same type)
//     double alpha_tilde = m_edge_compliance / pow(dts, 2);
    
//     // Loop through all the edge contraints
//     for (size_t i = 0; i < m_edge_contstraints_num; i++)    
//     {
//         // Get particle indices
//         size_t idx0 = m_edges_ids(i, 0);
//         size_t idx1 = m_edges_ids(i, 1);

//         // Get position vectors
//         Eigen::Vector3d x0 = x_tilde.row(idx0).transpose();
//         Eigen::Vector3d x1 = x_tilde.row(idx1).transpose();

//         // Get the original length of the edge
//         double l0_i = m_edge_lengths(i);

//         // Position norm
//         double l_i = (x0 - x1).norm();

//         // Define constrain
//         double ce = l_i - l0_i;

//         // Get inverted masses of associated particles
//         double w0 = m_inv_masses(idx0);
//         double w1 = m_inv_masses(idx1);

//         // Define gradients
//         Eigen::RowVector3d dce_dx0 = (x0 - x1).transpose() / l_i;
//         Eigen::RowVector3d dce_dx1 = - (x0 - x1).transpose() / l_i;

//         // Define weighted sum of gradients
//         double wsg = w0 + w1;

//         // Lagrange denominator
//         double lagrange_den = wsg + alpha_tilde;

//         if (lagrange_den < 1e-6)
//             return;
        
//         // Define lagrange correction
//         double delta_lambda = - (ce + alpha_tilde * lambda_edge(i)) /
//             lagrange_den;

//         // Update positions
//         x_tilde.row(idx0) += (w0 * dce_dx0.transpose() * delta_lambda).transpose();
//         x_tilde.row(idx1) += (w1 * dce_dx1.transpose() * delta_lambda).transpose();

//         // Update lagrange multipliers
//         lambda_edge(i) += delta_lambda;
//     }
// }

// // Solve volume contraints
// void st::ExtendedPBD::solve_volume_constraints(double dts, Eigen::MatrixXd&
//     x_tilde, Eigen::VectorXd& lambda_volume)
// {
//     // Calculate alpha (same for all contraints of the same type)
//     double alpha_tilde = m_vol_compliance / pow(dts, 2);

//     // Loop through all the tetrahedron elements
//     for (size_t i = 0; i < m_volume_constraints_num; i++)    
//     {
//         // Get particle indices
//         size_t idx0 = m_tet_mesh.elements(i, 0);
//         size_t idx1 = m_tet_mesh.elements(i, 1);
//         size_t idx2 = m_tet_mesh.elements(i, 2);
//         size_t idx3 = m_tet_mesh.elements(i, 3);
        
//         // Get particle coordinates
//         Eigen::Vector3d xa = x_tilde.row(idx0).transpose();
//         Eigen::Vector3d xb = x_tilde.row(idx1).transpose();
//         Eigen::Vector3d xc = x_tilde.row(idx2).transpose();
//         Eigen::Vector3d xd = x_tilde.row(idx3).transpose();

//         // Evaluate constraint
//         double ci =  6.0 * (tetrahedron_volume(xa, xb, xc, xd) - m_rest_volume(i));

//         // Calculate gradients
//         Eigen::RowVector3d dci_dxa = ((xd - xb).cross(xc - xb)).transpose();
//         Eigen::RowVector3d dci_dxb = ((xc - xa).cross(xd - xa)).transpose();
//         Eigen::RowVector3d dci_dxc = ((xd - xa).cross(xb - xa)).transpose();
//         Eigen::RowVector3d dci_dxd = ((xb - xa).cross(xc - xa)).transpose();

//         // Get inverted masses of associated particles
//         double wa = m_inv_masses(idx0);
//         double wb = m_inv_masses(idx1);
//         double wc = m_inv_masses(idx2);
//         double wd = m_inv_masses(idx3);
        
//         // Weighted sum of gradients
//         double wsg = wa * dci_dxa.squaredNorm() +
//             wb * dci_dxb.squaredNorm() + wc * dci_dxc.squaredNorm() +
//             wd * dci_dxd.squaredNorm();

//         // Langrange denominator
//         double lagrange_den = wsg + alpha_tilde;

//         if (lagrange_den < 1.e-5)
//             return;

//         // Calculate delta lagrange
//         double delta_lagrange = - (ci + alpha_tilde * lambda_volume(i)) /
//             (lagrange_den);

//         // Update positions
//         x_tilde.row(idx0) += (wa * dci_dxa.transpose() * delta_lagrange).transpose();
//         x_tilde.row(idx1) += (wb * dci_dxb.transpose() * delta_lagrange).transpose();
//         x_tilde.row(idx2) += (wc * dci_dxc.transpose() * delta_lagrange).transpose();
//         x_tilde.row(idx3) += (wd * dci_dxd.transpose() * delta_lagrange).transpose();

//         // Update lagrange multipliners
//         lambda_volume(i) += delta_lagrange;
//     }
// }

// // Solve boundary conditions
// void st::ExtendedPBD::solve_boundary_constraints(double real_time, double dts, 
//     Eigen::MatrixXd& x_tilde, Eigen::VectorXd& lambda_bc)
// {
//     // Calculate alpha (same for all contraints of the same type)
//     double alpha_tilde = m_bc_compliance / pow(dts, 2);

//     // Loop through all boundary points
//     for (size_t i = 0; i < m_bc_constraints_num; i++)
//     {
//         // Get index of point a
//         size_t idx_a = m_boundary_points_indices(i);
        
//         // Get position of point a
//         Eigen::Vector3d xa = m_tet_mesh.vertices.row(idx_a).transpose();
        
//         // Set boundaries
//         Eigen::Vector3d da = m_initial_tet_mesh.vertices.row(idx_a).transpose();

//         // Evaluate constraint
//         double ci =  (xa - da).squaredNorm();

//         // Calculate gradients
//         Eigen::RowVector3d dci_dxa = 2.0 * (xa - da).transpose();

//         // Get inverted masses of associated particles
//         double wa = m_inv_masses(idx_a);

//         // Weighted sum of gradients
//         double wsg = wa * dci_dxa.squaredNorm();

//         // Lagrange denominator
//         double lagrange_den = wsg + alpha_tilde;

//         if (lagrange_den < 1.0e-6) 
//             return;

//         // Calculate delta lagrange
//         double delta_lagrange = - (ci + alpha_tilde * lambda_bc(i)) / lagrange_den;

//         // Update positions
//         x_tilde.row(idx_a) += (wa * dci_dxa.transpose() * delta_lagrange).transpose();
    
//         // Update lambdas
//         lambda_bc(i) += delta_lagrange;
//     } 
// }

// // Solve needle tissue interaction constraints
// void st::ExtendedPBD::solve_needle_tissue_interation_constraints(const
//     std::vector<NeedleCollisionInfo>& collision_info,
//     const std::vector<int>& active_constraints_indices,
//     double dts, Eigen::MatrixXd& x_tilde, Eigen::VectorXd& lambda_nt)
// {

//     // Calculate alpha (same for all contraints of the same type)
//     double alpha_tilde = m_nt_compliance / pow(dts, 2);
    
//     // Loop through all boundary points
//     for (size_t i = 0; i < active_constraints_indices.size(); i++)
//     {
//         // Active constaints indices
//         int active_idx = active_constraints_indices.at(i);
        
//         // Vector roa_f_f_0
//         Eigen::Vector3d roa_F_F =
//             collision_info.at(active_idx).point_inertial_position;

//         // Get tet id        
//         int tet_id = collision_info.at(active_idx).tet_element_id;

//         // Get particle indices
//         size_t idx0 = m_tet_mesh.elements(tet_id, 0);
//         size_t idx1 = m_tet_mesh.elements(tet_id, 1);
//         size_t idx2 = m_tet_mesh.elements(tet_id, 2);
//         size_t idx3 = m_tet_mesh.elements(tet_id, 3);

//         // Get particles inertial positions
//         Eigen::Vector3d xa = m_tet_mesh.vertices.row(idx0).transpose();
//         Eigen::Vector3d xb = m_tet_mesh.vertices.row(idx1).transpose();
//         Eigen::Vector3d xc = m_tet_mesh.vertices.row(idx2).transpose();
//         Eigen::Vector3d xd = m_tet_mesh.vertices.row(idx3).transpose();

//         // Vector rob_f_f
//         Eigen::Vector3d rob_F_F = 0.25 * (xa + xb + xc + xd);

//         // Vector rab_f_f
//         Eigen::Matrix3d rot_f_F = Eigen::Matrix3d::Identity();
//         Eigen::Vector3d rab_f_f = rot_f_F.transpose() * (rob_F_F - roa_F_F);

//         // Vector rab_f_f_0
//         Eigen::Vector3d rab_f_f_0 = m_collision_array_rab_f_f_0.row(active_idx).transpose();

//         // Evaluate constraint
//         double ci =  (m_l_mat * (rab_f_f - rab_f_f_0)).squaredNorm();
        
//         // Calculate gradients (same for all points xa, xb, xc and xd)
//         Eigen::RowVector3d dci_dxk = 2.0 * (m_l_mat * (rab_f_f -
//             rab_f_f_0)).transpose() * 0.25 * rot_f_F.transpose();

//         // Get inverted masses of associated particles
//         double wa = m_inv_masses(idx0);
//         double wb = m_inv_masses(idx1);
//         double wc = m_inv_masses(idx2);
//         double wd = m_inv_masses(idx3);

//         // Weighted sum of gradients
//         double wsg = wa * dci_dxk.squaredNorm() + wb * dci_dxk.squaredNorm() + 
//             wc * dci_dxk.squaredNorm() + wd * dci_dxk.squaredNorm();

//         // Lagrange denominator
//         double lagrange_den = wsg + alpha_tilde;

//         if (lagrange_den < 1.0e-6) 
//             return;

//         // Calculate delta lagrange
//         double delta_lagrange = - (ci + alpha_tilde * lambda_nt(i)) / lagrange_den;

//         // Update positions
//         x_tilde.row(idx0) += (wa * dci_dxk.transpose() * delta_lagrange).transpose();
//         x_tilde.row(idx1) += (wb * dci_dxk.transpose() * delta_lagrange).transpose();
//         x_tilde.row(idx2) += (wc * dci_dxk.transpose() * delta_lagrange).transpose();
//         x_tilde.row(idx3) += (wd * dci_dxk.transpose() * delta_lagrange).transpose();
    
//         // Update lambdas
//         lambda_nt(i) += delta_lagrange;
//     } 
// }

// // Update collision arrays        
// void st::ExtendedPBD::update_nt_collision_array(const
//     std::vector<NeedleCollisionInfo>& collision_info, const std::vector<int>&
//     active_constraints_indices)
// {
//     // Check if the needle particle has moved to the next tetrahedron
//     for (size_t i = 0; i < active_constraints_indices.size(); i++)
//     {
//         // Active constaints indices
//         int active_idx = active_constraints_indices.at(i);

//         // Get tet id at active constraint
//         int tet_id = collision_info.at(active_idx).tet_element_id;

//         // Check for condition and update collision array
//         if ( tet_id != m_collision_array(active_idx))
//         {
//             // Update collision array
//             m_collision_array(active_idx) = tet_id;

//             // Update initial contact orientation 
//             //...

//             // Vector roa_f_f_0
//             Eigen::Vector3d roa_F_F_0 =
//                 collision_info.at(active_idx).point_inertial_position;

//             // Get particle indices
//             size_t idx0 = m_tet_mesh.elements(tet_id, 0);
//             size_t idx1 = m_tet_mesh.elements(tet_id, 1);
//             size_t idx2 = m_tet_mesh.elements(tet_id, 2);
//             size_t idx3 = m_tet_mesh.elements(tet_id, 3);

//             // Get particles inertial positions
//             Eigen::Vector3d xa_0 = m_tet_mesh.vertices.row(idx0).transpose();
//             Eigen::Vector3d xb_0 = m_tet_mesh.vertices.row(idx1).transpose();
//             Eigen::Vector3d xc_0 = m_tet_mesh.vertices.row(idx2).transpose();
//             Eigen::Vector3d xd_0 = m_tet_mesh.vertices.row(idx3).transpose();

//             // Vector rob_f_f_0
//             Eigen::Vector3d rob_F_F_0 = 0.25 * (xa_0 + xb_0 + xc_0 + xd_0);

//             // Vector rab_f_f_0
//             Eigen::Vector3d rab_f_f_0 =
//                 m_collision_array_rotation.at(i).transpose() * (rob_F_F_0 -
//                     roa_F_F_0);

//             // Update initial contact position
//             m_collision_array_rab_f_f_0.row(active_idx) = rab_f_f_0.transpose();
//         }
//     }
// }


// // Set point masses 
// void st::ExtendedPBD::set_point_masses(void)
// {
//     // Initialize masses
//     m_masses = Eigen::VectorXd::Zero(m_particles_num, 1);

//     // Initialize inverse masses
//     m_inv_masses = Eigen::VectorXd::Zero(m_particles_num, 1);

//     // // Set inverse masses      
//     // for (size_t i = 0; i < m_tets_num; i++)
//     // {
//     //     // Get indices of element i
//     //     size_t idx_0 = m_tet_mesh.elements(i, 0);
//     //     size_t idx_1 = m_tet_mesh.elements(i, 1);
//     //     size_t idx_2 = m_tet_mesh.elements(i, 2);
//     //     size_t idx_3 = m_tet_mesh.elements(i, 3);

//     //     // Get vertices of element i
//     //     Eigen::Vector3d p0 = m_tet_mesh.vertices.row(idx_0).transpose();
//     //     Eigen::Vector3d p1 = m_tet_mesh.vertices.row(idx_1).transpose();
//     //     Eigen::Vector3d p2 = m_tet_mesh.vertices.row(idx_2).transpose();
//     //     Eigen::Vector3d p3 = m_tet_mesh.vertices.row(idx_3).transpose();

//     //     // Calculate volume of tetrahedron i
//     //     double tet_i_vol = tetrahedron_volume(p0, p1, p2, p3);

//     //     // Set rest volume
//     //     m_rest_volume(i) = tet_i_vol;

//     //     // Set inverse mass i
//     //     double p_inv_mass = tet_i_vol > 0.0 ? (1.0 / (tet_i_vol / 4.0)) : 0.0;

//     //     // Set inverse mass container - adds p_inv_mass to the vertices of
//     //     // the tetrahedron
//     //     m_inv_masses(idx_0) += p_inv_mass;
//     //     m_inv_masses(idx_1) += p_inv_mass;
//     //     m_inv_masses(idx_2) += p_inv_mass;
//     //     m_inv_masses(idx_3) += p_inv_mass;
//     // }

//     // Get masses 
//     for (size_t i = 0; i < m_inv_masses.size(); i++)
//     {
//         m_masses(i) = 1;
//         m_inv_masses(i) = 1.0 / m_masses(i);
//     }
    
// }

