#include "../include/xpbd_constraints.h"


// Calculate tetrahedron volume
double XPBDConstraints::tetrahedron_volume(const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
    const Eigen::Vector3d& p3)
{
    double vol = (1. / 6.) * (p1 - p0).cross(p2 - p0).dot(p3 - p0);
    return std::abs(vol);
}

// Solve edge contraints
void XPBDConstraints::solve_edge_constraints(double dts,
    const Eigen::VectorXd& inv_masses, const Eigen::VectorXd& edge_lengths, 
    double edge_compliance, const Eigen::MatrixXi& edges_ids,
    Eigen::MatrixXd& x_tilde, Eigen::VectorXd& lambda_edge)
{
    // Calculate alpha (same for all contraints of the same type)
    double alpha_tilde = edge_compliance / pow(dts, 2);
    
    // Loop through all the edge contraints
    for (size_t i = 0; i < edges_ids.rows(); i++)    
    {
        // Get particle indices
        size_t idx0 = edges_ids(i, 0);
        size_t idx1 = edges_ids(i, 1);

        // Get position vectors
        Eigen::Vector3d x0 = x_tilde.row(idx0).transpose();
        Eigen::Vector3d x1 = x_tilde.row(idx1).transpose();

        // Get the original length of the edge
        double l0_i = edge_lengths(i);

        // Position norm
        double l_i = (x0 - x1).norm();

        // Define constrain
        double ce = l_i - l0_i;

        // Get inverted masses of associated particles
        double w0 = inv_masses(idx0);
        double w1 = inv_masses(idx1);

        // Define gradients
        Eigen::RowVector3d dce_dx0 = (x0 - x1).transpose() / l_i;
        Eigen::RowVector3d dce_dx1 = - (x0 - x1).transpose() / l_i;

        // Define weighted sum of gradients
        double wsg = w0 + w1;

        // Lagrange denominator
        double lagrange_den = wsg + alpha_tilde;

        if (lagrange_den < 1e-6)
            return;
        
        // Define lagrange correction
        double delta_lambda = - (ce + alpha_tilde * lambda_edge(i)) /
            lagrange_den;

        // Update positions
        x_tilde.row(idx0) += (w0 * dce_dx0.transpose() * delta_lambda).transpose();
        x_tilde.row(idx1) += (w1 * dce_dx1.transpose() * delta_lambda).transpose();

        // Update lagrange multipliers
        lambda_edge(i) += delta_lambda;
    }
}

// Solve volume constraints
void XPBDConstraints::solve_tet_volume_constraints(double dts,
    const Eigen::VectorXd& inv_masses, const Eigen::VectorXd& rest_volume,
    double volume_compliance, const Eigen::MatrixXi& tet_elements,
    Eigen::MatrixXd& x_tilde, Eigen::VectorXd& lambda_volume)
{
    // Calculate alpha (same for all contraints of the same type)
    double alpha_tilde = volume_compliance / pow(dts, 2);

    // Loop through all the tetrahedron elements
    for (size_t i = 0; i < rest_volume.rows(); i++)    
    {
        // Get particle indices
        size_t idx0 = tet_elements(i, 0);
        size_t idx1 = tet_elements(i, 1);
        size_t idx2 = tet_elements(i, 2);
        size_t idx3 = tet_elements(i, 3);

        // Get particle coordinates
        Eigen::Vector3d xa = x_tilde.row(idx0).transpose();
        Eigen::Vector3d xb = x_tilde.row(idx1).transpose();
        Eigen::Vector3d xc = x_tilde.row(idx2).transpose();
        Eigen::Vector3d xd = x_tilde.row(idx3).transpose();

        // Evaluate constraint
        double ci =  6.0 * (XPBDConstraints::tetrahedron_volume(xa, xb, xc, xd)
            - rest_volume(i));

        // Calculate gradients
        Eigen::RowVector3d dci_dxa = ((xd - xb).cross(xc - xb)).transpose();
        Eigen::RowVector3d dci_dxb = ((xc - xa).cross(xd - xa)).transpose();
        Eigen::RowVector3d dci_dxc = ((xd - xa).cross(xb - xa)).transpose();
        Eigen::RowVector3d dci_dxd = ((xb - xa).cross(xc - xa)).transpose();

        // Get inverted masses of associated particles
        double wa = inv_masses(idx0);
        double wb = inv_masses(idx1);
        double wc = inv_masses(idx2);
        double wd = inv_masses(idx3);

        // Weighted sum of gradients
        double wsg = wa * dci_dxa.squaredNorm() +
            wb * dci_dxb.squaredNorm() + wc * dci_dxc.squaredNorm() +
            wd * dci_dxd.squaredNorm();

        // Langrange denominator
        double lagrange_den = wsg + alpha_tilde;

        if (lagrange_den < 1.e-5)
            return;

        // Calculate delta lagrange
        double delta_lagrange = - (ci + alpha_tilde * lambda_volume(i)) /
            (lagrange_den);

        // Update positions
        x_tilde.row(idx0) += (wa * dci_dxa.transpose() * delta_lagrange).transpose();
        x_tilde.row(idx1) += (wb * dci_dxb.transpose() * delta_lagrange).transpose();
        x_tilde.row(idx2) += (wc * dci_dxc.transpose() * delta_lagrange).transpose();
        x_tilde.row(idx3) += (wd * dci_dxd.transpose() * delta_lagrange).transpose();

        // Update lagrange multipliners
        lambda_volume(i) += delta_lagrange;
    }
}

// Solve boundary constraints
void XPBDConstraints::solve_point_boundary_constraints_single(double dts, 
    const Eigen::VectorXd& inv_masses, 
    const Eigen::MatrixXd& desired_vertices_positions,
    double boundary_compliance,
    const Eigen::VectorXi& boundary_points_indices,
    Eigen::MatrixXd& x_tilde,
    Eigen::VectorXd& lambda_boundary)
{
    // Calculate alpha (same for all contraints of the same type)
    double alpha_tilde = boundary_compliance / pow(dts, 2);

    // Loop through all boundary points
    for (size_t i = 0; i < boundary_points_indices.rows(); i++)
    {
        // Get index of point a
        size_t idx_a = boundary_points_indices(i);

        // Get position of point a
        Eigen::Vector3d xa = x_tilde.row(idx_a).transpose();

        // Set boundaries
        Eigen::Vector3d da = desired_vertices_positions.row(idx_a).transpose();

        // Evaluate constraint
        double ci =  (xa - da).squaredNorm();

        // Calculate gradients
        Eigen::RowVector3d dci_dxa = 2.0 * (xa - da).transpose();

        // Get inverted masses of associated particles
        double wa = inv_masses(idx_a);

        // Weighted sum of gradients
        double wsg = wa * dci_dxa.squaredNorm();

        // Lagrange denominator
        double lagrange_den = wsg + alpha_tilde;

        if (lagrange_den < 1.0e-6) 
            return;

        // Calculate delta lagrange
        double delta_lagrange = - (ci + alpha_tilde * lambda_boundary(i)) / lagrange_den;

        // Update positions
        x_tilde.row(idx_a) += (wa * dci_dxa.transpose() * delta_lagrange).transpose();

        // Update lambdas
        lambda_boundary(i) += delta_lagrange;
    }
}

// Solve boundary constraints
void XPBDConstraints::solve_point_boundary_constraints(double dts, 
    const Eigen::VectorXd& inv_masses,
    const Eigen::Matrix3d& boundary_compliance,
    const Eigen::MatrixXd& desired_positions,
    const Eigen::VectorXi& boundary_points_indices,
    Eigen::MatrixXd& x_tilde, std::vector<Eigen::Vector3d>& lambda_boundary)
{
    // Calculate alpha tilde matrix
    Eigen::Matrix3d alpha_tilde = (1.0 / pow(dts, 2)) * boundary_compliance;

    for (size_t i = 0; i < boundary_points_indices.rows(); i++)
    {
        // Define idx1
        int idx1 = boundary_points_indices(i);

        // Get inverse masses
        double inv_mass1 = inv_masses(idx1);

        // Define position
        Eigen::Vector3d x1 = x_tilde.row(idx1).transpose();
        
        // Define desired position
        Eigen::Vector3d xd = desired_positions.row(idx1).transpose();
        
        // Define winkler constraint
        Eigen::Vector3d cw =  (x1 - xd);

        // Calculate gradients with respect to positions
        Eigen::Matrix3d dcw_dx1 = Eigen::Matrix3d::Identity();

        // Specify weighted sum of gradients
        Eigen::Matrix3d wsg =  dcw_dx1 * inv_mass1 * dcw_dx1.transpose();
            
        // Equation right hand side
        Eigen::Vector3d erhs = - cw - alpha_tilde * lambda_boundary.at(i);

        // Delta lagrange
        Eigen::Vector3d delta_lagrange = ( wsg +
            alpha_tilde).colPivHouseholderQr().solve(erhs);

        // Update lagrange
        lambda_boundary.at(i) += delta_lagrange;

        // Find corrections
        x_tilde.row(idx1) += (inv_mass1 * dcw_dx1.transpose() * delta_lagrange).transpose();
    }
}

// Solve quaternion boundary constraints
void XPBDConstraints::solve_quaternion_boundary_constraints(double dts, 
    const std::vector<Eigen::Matrix3d>& inertia_tensors,
    const std::vector<Eigen::Quaterniond>& desired_quaternions,
    const Eigen::Matrix3d& boundary_compliance,
    const Eigen::VectorXi& boundary_quaternion_indices,
    std::vector<Eigen::Quaterniond>& q_tilde,
    std::vector<Eigen::Vector3d>& lambda_bound)
{
    // Calculate alpha 
    Eigen::Matrix3d alpha_tilde = (1.0 / pow(dts, 2)) * boundary_compliance;

    // Loop through all boundary quaternions
    for (size_t i = 0; i < boundary_quaternion_indices.rows(); i++)
    {
        // Get index of point a
        size_t idx_a = boundary_quaternion_indices(i);

        // Get quaternion q
        Eigen::Quaterniond q = q_tilde.at(idx_a);
        
        // Get desired quaternion qd inverse
        Eigen::Quaterniond qd = desired_quaternions.at(idx_a);
        
        // Define the inverse of quaternion qd
        Eigen::Quaterniond p = qd.inverse();

        // Evaluate constraint
        Eigen::Vector3d c = (q * p).vec();
        
	    // Define gradients with respect to rotation
	    Eigen::MatrixXd dc_dq = Eigen::MatrixXd::Zero(3, 4);

        dc_dq.col(0) = p.vec();
        dc_dq.block(0, 1, 3, 3) = p.w() * Eigen::Matrix3d::Identity() -
            dme::S(p.vec());
        
        // Get inertia for body 0
        Eigen::Matrix3d inertia0_body = inertia_tensors.at(idx_a);
        
 	    // Approxmate inverse inertia
	    double w_in = 1.0 / inertia0_body(1, 1);
	
	    // Specify weighted sum of gradients
	    Eigen::Matrix3d wsg = dc_dq * w_in * dc_dq.transpose();

	    // Delta lagrange
	    Eigen::Vector3d delta_lagrange = (wsg +
            alpha_tilde).colPivHouseholderQr().solve(- c - alpha_tilde *
            lambda_bound.at(i));

        // Update lagrange
        lambda_bound.at(i) += delta_lagrange;

	    // Find corrections
	    Eigen::Vector4d corr_q_vec = w_in * dc_dq.transpose() * delta_lagrange;

	    q_tilde.at(idx_a).coeffs() += Eigen::Quaterniond(corr_q_vec(0),
            corr_q_vec(1), corr_q_vec(2), corr_q_vec(3)).coeffs();
    }
}

// Cosserat shear and stretch constraints
void XPBDConstraints::cosserat_stretch_shear_constraints(
    double dts, const Eigen::VectorXd& inv_masses,
    const std::vector<Eigen::Matrix3d>& inertia_tensors,
    double element_length, const Eigen::Matrix3d& stretch_compliance, 
    const Eigen::MatrixXd& pcp,
    Eigen::MatrixXd& x_tilde,
    std::vector<Eigen::Quaterniond>& q_tilde,
    std::vector<Eigen::Vector3d>& lambda_stretch)
{
    // Calculate alpha tilde matrix
    Eigen::Matrix3d alpha_tilde = (1.0 / pow(dts, 2)) * stretch_compliance;
    
	// // Epsilon 
	// double eps = 1.e-6;

	// Inverse element length
	double inv_element_length = 1.0 / element_length;

	// Define e1 as quaternion
	Eigen::Quaterniond e1_quat = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);

    #pragma omp parallel num_threads(4)
    {
        // Loop through all needle particles connection pairs and apply corrections
        #pragma omp for
        for (size_t i = 0; i < pcp.rows(); i++)
        {
            // Get connection pair i
            int idx0 = pcp(i, 0); int idx1 = pcp(i, 1);    

            // Find global positions 
            Eigen::Vector3d x0 = x_tilde.row(idx0).transpose();
            Eigen::Vector3d x1 = x_tilde.row(idx1).transpose(); 

            // Define quaternions indices
            int idx0_quat = idx0 - 1;

            // Correct quaternion indices
            if (idx0_quat < 0) { idx0_quat = 0;}

            // Find global orientations 
            Eigen::Quaterniond q0 = q_tilde.at(idx0_quat);

            // Get inverse masses
            double inv_mass0 = inv_masses(idx0);
            double inv_mass1 = inv_masses(idx1);
        
            // Get inertia for body 0
            Eigen::Matrix3d inertia0_body = inertia_tensors.at(idx0_quat);

	        // First director d1 = q0 * e1_quat * q0_conjugate
	        Eigen::Vector3d d1 = (q0 * e1_quat * q0.conjugate()).vec();

	        // Define stretch costrain
	        Eigen::Vector3d cs = inv_element_length * (x1 - x0) - d1;

	        // Calculate gradients with respect to positions
	        Eigen::Matrix3d dcs_dx0 = - inv_element_length * Eigen::Matrix3d::Identity();
	        Eigen::Matrix3d dcs_dx1 = inv_element_length * Eigen::Matrix3d::Identity();

	        // Calculate gradients with respect to rotation
	        Eigen::Quaterniond d_quat = e1_quat * q0.conjugate();

	        // Initialize gradients with respect to rotation
	        Eigen::MatrixXd dcs_dq0 = Eigen::MatrixXd::Zero(3, 4);

	        // Define gradients with respect to roration
	        dcs_dq0.col(0) = d_quat.vec();

	        dcs_dq0.block(0, 1, 3, 3) = d_quat.w() * Eigen::Matrix3d::Identity() - 
		        dme::S(d_quat.vec());
	        dcs_dq0 *= -2.0;

 	        // Approxmate inverse inertia
	        double w_in = 1.0 / inertia0_body(1, 1);
	
	        // Specify weighted sum of gradients
	        Eigen::Matrix3d wsg = dcs_dx0 * inv_mass0 * dcs_dx0.transpose() + 
	    	    dcs_dx1 * inv_mass1 * dcs_dx1.transpose() + 
	    	    dcs_dq0 * w_in * dcs_dq0.transpose();

            // Equation right hand side
            Eigen::Vector3d erhs = - cs - alpha_tilde * lambda_stretch.at(i);

	        // Delta lagrange
	        Eigen::Vector3d delta_lagrange = ( wsg +
                alpha_tilde).colPivHouseholderQr().solve(erhs);

            // Update lagrange
            lambda_stretch.at(i) += delta_lagrange;

	        // Find corrections
	        x_tilde.row(idx0) += (inv_mass0 * dcs_dx0.transpose() * delta_lagrange).transpose();

	        // Find corrections
	        x_tilde.row(idx1) += (inv_mass1 * dcs_dx1.transpose() * delta_lagrange).transpose();
        
	        // Find corrections
	        Eigen::Vector4d corr_q0_vec = w_in * dcs_dq0.transpose() * delta_lagrange;

	        q_tilde.at(idx0_quat).coeffs() += Eigen::Quaterniond(corr_q0_vec(0),
                corr_q0_vec(1), corr_q0_vec(2), corr_q0_vec(3)).coeffs();
        }
    }
}

/** Determine the position corrections for the bending and torsion
 * constraint constraint (eq. 40 in the paper).
*/
void XPBDConstraints::cosserat_bend_twist_constraints(double dts,
    const std::vector<Eigen::Matrix3d>& inertia_tensors,
    const double element_length,
    const Eigen::Matrix3d& bending_compliance, 
    const Eigen::Vector3d& rest_darboux_vector,
    const Eigen::MatrixXd& ecp,
    std::vector<Eigen::Quaterniond>& q_tilde,
    std::vector<Eigen::Vector3d>& lambda_bend)
{
    // Calculate alpha 
    Eigen::Matrix3d alpha_tilde = (1.0 / pow(dts, 2)) * bending_compliance;

	// Epsilon 
	double eps = 1.e-6;

    // Loop through all needle element connection pairs and apply corrections
    #pragma omp parallel num_threads(4)
    {
        #pragma omp for
        for (size_t i = 0; i < ecp.rows(); i++)
        {
            // Get connection pair i
            int idx0 = ecp(i, 0);
            int idx1 = ecp(i, 1);    

            // Find global orientations 
            Eigen::Quaterniond q0 = q_tilde.at(idx0);
            Eigen::Quaterniond q1 = q_tilde.at(idx1);

            // Get inertia for bodies 0 and 1
            Eigen::Matrix3d inertia0_body = inertia_tensors.at(idx0);
            Eigen::Matrix3d inertia1_body = inertia_tensors.at(idx1);

            // Approximate inverse inertial
            double w_in0 = 1.0 / inertia0_body(1, 1);
            double w_in1 = 1.0 / inertia1_body(1, 1);

            // Calculate darboux vector
            Eigen::Vector3d darboux_vec = calculate_darboux_vector(q0, q1, element_length);

            // Quaternion darboux_plus
            Eigen::Vector3d darboux_plus = darboux_vec + rest_darboux_vector;

            // Quaternion darboux_diff
            Eigen::Vector3d darboux_diff = darboux_vec - rest_darboux_vector;

            // Define s
            double s = 1;
            if (darboux_diff.squaredNorm() > darboux_plus.squaredNorm()) { s = -1; }

            // Define bending/twist constraint
            Eigen::Vector3d cb = darboux_vec - s * rest_darboux_vector;

            // Calculate gradients with respect to first quaternion
            Eigen::MatrixXd dcb_dq0 = Eigen::MatrixXd::Zero(3, 4);
            dcb_dq0.col(0) = - q1.vec();
            dcb_dq0.block(0, 1, 3, 3) =  q1.w() * Eigen::Matrix3d::Identity() - 
                dme::S(q1.vec());
            dcb_dq0 *= - (2.0 / element_length);

            // Calculate gradients with respect to second quaternion
            Eigen::MatrixXd dcb_dq1 = Eigen::MatrixXd::Zero(3, 4);
            dcb_dq1.col(0) = - q0.vec();
            dcb_dq1.block(0, 1, 3, 3) =  q0.w() * Eigen::Matrix3d::Identity() - 
                dme::S(q0.vec());
            dcb_dq1 *= (2.0 / element_length);

            // Specify weighted sum of gradients
            Eigen::Matrix3d wsg = dcb_dq0 * w_in0 * dcb_dq0.transpose() + 
                dcb_dq1 * w_in1 * dcb_dq1.transpose() + 
                eps * Eigen::Matrix3d::Identity();   

            // Delta lagrange
            Eigen::Vector3d delta_lagrange = (wsg +
                alpha_tilde).colPivHouseholderQr().solve(- cb - alpha_tilde *
                    lambda_bend.at(i));

            // Update lagrange
            lambda_bend.at(i) += delta_lagrange;

            // Find corrections for element0
            Eigen::Vector4d corr_q0_vec = w_in0 * dcb_dq0.transpose() * delta_lagrange;

            q_tilde.at(idx0).coeffs() += Eigen::Quaterniond(corr_q0_vec(0),
                corr_q0_vec(1), corr_q0_vec(2), corr_q0_vec(3)).coeffs();
	
            // Find corrections for element1
            Eigen::Vector4d corr_q1_vec = w_in1 * dcb_dq1.transpose() * delta_lagrange;

            q_tilde.at(idx1).coeffs() += Eigen::Quaterniond(corr_q1_vec(0),
                corr_q1_vec(1), corr_q1_vec(2), corr_q1_vec(3)).coeffs();
        }
    }
}

// Needle tissue interaction lateral constraints
void XPBDConstraints::needle_tissue_interaction_lateral_constraints(double dts, 
    const Eigen::VectorXd& tissue_inv_masses,
    const Eigen::VectorXd& needle_inv_masses,
    const std::vector<Eigen::Matrix3d>& needle_inertia_tensors,
    double nt_compliance,
    const std::vector<int>& active_constraints_indices,
    const std::vector<CollisionsInterface::NeedleCollisionInfo>& collision_info,
    const Eigen::MatrixXd& initial_collision_array,
    const Eigen::MatrixXi& tissue_mesh_elements,
    const Eigen::Matrix3d& selection_matrix,
    Eigen::MatrixXd& x_tissue_tilde,
    Eigen::MatrixXd& x_needle_tilde,
    std::vector<Eigen::Quaterniond>& q_needle_tilde,
    Eigen::VectorXd& lambda_lateral_nti)
{
    // Calculate alpha (same for all contraints of the same type)
    double alpha_tilde = nt_compliance / pow(dts, 2);
    
    // Exclude tip from the constraints
    int indices_num = active_constraints_indices.size();

    // Loop through all active constraints
    for (size_t i = 0; i < std::max(indices_num, 0); i++)
    {
        // Active constaints indices
        int active_idx = active_constraints_indices.at(i);
        
        // Vector roa_f_f
        Eigen::Vector3d roa_F_F = x_needle_tilde.row(active_idx).transpose();
    
        // Get tet id        
        int tet_id = collision_info.at(active_idx).tet_element_id;

        // Define quaternions indices
        int quat_idx = active_idx - 1;

        // Correct quaternion indices
        if (quat_idx < 0) { quat_idx = 0; }

        // Find global orientations 
        Eigen::Quaterniond qa = q_needle_tilde.at(quat_idx);
            
        // Get rotation matrix of element j wrt to F
        Eigen::Matrix3d rot_f_F = qa.toRotationMatrix();

        // Get particle indices
        size_t idx0_tissue = tissue_mesh_elements(tet_id, 0);
        size_t idx1_tissue = tissue_mesh_elements(tet_id, 1);
        size_t idx2_tissue = tissue_mesh_elements(tet_id, 2);
        size_t idx3_tissue = tissue_mesh_elements(tet_id, 3);

        // Get particles inertial positions
        Eigen::Vector3d xa_tissue = x_tissue_tilde.row(idx0_tissue).transpose();
        Eigen::Vector3d xb_tissue = x_tissue_tilde.row(idx1_tissue).transpose();
        Eigen::Vector3d xc_tissue = x_tissue_tilde.row(idx2_tissue).transpose();
        Eigen::Vector3d xd_tissue = x_tissue_tilde.row(idx3_tissue).transpose();

        // Vector rob_f_f
        Eigen::Vector3d rob_F_F = 0.25 * (xa_tissue + xb_tissue + xc_tissue +
            xd_tissue);

        // Define p vector
        Eigen::Vector3d p_vec = rob_F_F - roa_F_F;

        // Vector rab_f_f
        Eigen::Vector3d rab_f_f = rot_f_F.transpose() * p_vec;

        // Vector rab_f_f_0 or a
        Eigen::Vector3d a =
            initial_collision_array.row(active_idx).transpose();

        // Define s scalar
        double s = (selection_matrix * a).squaredNorm();

        // Evaluate constraint
        double ck =  (selection_matrix * rab_f_f).squaredNorm() - s;
        
        // Define cr vector
        Eigen::RowVector3d cr = 2.0 * rab_f_f.transpose() *
            selection_matrix.transpose();
        
        // Calculate gradients with respect to the tissue partices (same for all tissue particles)
        Eigen::RowVector3d dck_dxm_tissue = cr * rot_f_F.transpose() * 0.25;

        // Calculate gradients with respect to the needle particles
        Eigen::RowVector3d dck_dna = - cr * rot_f_F.transpose();

        // Calculate gradients with repsect to the needle rotation
        Eigen::MatrixXd w_mat = Eigen::MatrixXd::Zero(3, 4);
        w_mat.col(0) = 2.0 * qa.w() * p_vec + 2.0 * dme::S(p_vec) * qa.vec();
        w_mat.block(0, 1, 3, 3) = 2.0 * qa.vec().transpose() * p_vec *
            Eigen::Matrix3d::Identity() + 2.0 * qa.vec() * p_vec.transpose() - 
            2.0 * p_vec * qa.vec().transpose() + 2.0 * qa.w() * dme::S(p_vec);

        Eigen::RowVector4d dck_dqa = - cr * w_mat;

        // Get inverse masses of associated tissue particles
        double wa_tissue = tissue_inv_masses(idx0_tissue);
        double wb_tissue = tissue_inv_masses(idx1_tissue);
        double wc_tissue = tissue_inv_masses(idx2_tissue);
        double wd_tissue = tissue_inv_masses(idx3_tissue);

        // Get inverse masses of associated needle particles
        double wp_needle = needle_inv_masses(active_idx);

        // Get inertia for bodies 0 
        Eigen::Matrix3d inertia0_body = needle_inertia_tensors.at(quat_idx);

        // Approximate inverse inertial
        double wq_needle = 1.0 / inertia0_body(1, 1);
        
        // Weighted sum of gradients
        double wsg = wa_tissue * dck_dxm_tissue.squaredNorm() +
            wb_tissue * dck_dxm_tissue.squaredNorm() + 
            wc_tissue * dck_dxm_tissue.squaredNorm() +
            wd_tissue * dck_dxm_tissue.squaredNorm() + 
            wp_needle * dck_dna.squaredNorm() +
            dck_dqa * wq_needle * dck_dqa.transpose();
            
        // Lagrange denominator
        double lagrange_den = wsg + alpha_tilde;

        // Calculate delta lagrange
        double delta_lagrange = - (ck + alpha_tilde * lambda_lateral_nti(i)) /
            lagrange_den;

        // Update lagrange
        lambda_lateral_nti(i) += delta_lagrange;

        // Update positions
        x_tissue_tilde.row(idx0_tissue) += (wa_tissue *
            dck_dxm_tissue.transpose() * delta_lagrange).transpose();

        x_tissue_tilde.row(idx1_tissue) += (wb_tissue *
            dck_dxm_tissue.transpose() * delta_lagrange).transpose();

        x_tissue_tilde.row(idx2_tissue) += (wc_tissue *
            dck_dxm_tissue.transpose() * delta_lagrange).transpose();

        x_tissue_tilde.row(idx3_tissue) += (wd_tissue *
            dck_dxm_tissue.transpose() * delta_lagrange).transpose();
    
        x_needle_tilde.row(active_idx) += (wp_needle * 
            dck_dna.transpose() * delta_lagrange).transpose();
    
        // Find corrections for element0
        Eigen::Vector4d corr_q0_vec = wq_needle * dck_dqa.transpose() * delta_lagrange;

        q_needle_tilde.at(quat_idx).coeffs() += Eigen::Quaterniond(corr_q0_vec(0),
            corr_q0_vec(1), corr_q0_vec(2), corr_q0_vec(3)).coeffs();
    }        
}

// Needle tissue interaction lateral constraints multi
void XPBDConstraints::needle_tissue_interaction_lateral_constraints_3d(
    double dts, 
    const Eigen::VectorXd& tissue_inv_masses,
    const Eigen::VectorXd& needle_inv_masses,
    const std::vector<Eigen::Matrix3d>& needle_inertia_tensors,
    double nt_compliance,
    const std::vector<int>& active_constraints_indices,
    const std::vector<CollisionsInterface::NeedleCollisionInfo>& collision_info,
    const Eigen::MatrixXd& initial_collision_array,
    const std::vector<Eigen::Matrix3d>& initial_collision_rot_array,
    const Eigen::MatrixXi& tissue_mesh_elements,
    const Eigen::Matrix3d& selection_matrix,
    Eigen::MatrixXd& x_tissue_tilde,
    Eigen::MatrixXd& x_needle_tilde,
    std::vector<Eigen::Quaterniond>& q_needle_tilde,
    std::vector<Eigen::Vector3d>& lambda_lateral_nti)
{
    // Calculate alpha (same for all contraints of the same type)
    Eigen::Matrix3d alpha_tilde = (nt_compliance / pow(dts, 2)) *
        Eigen::Matrix3d::Identity();
    
    // Exclude tip from the constraints
    int indices_num = active_constraints_indices.size();

    // Loop through all active constraints
    for (size_t i = 0; i < std::max(indices_num, 0); i++)
    {
        // Active constaints indices
        int active_idx = active_constraints_indices.at(i);
        
        // Vector roa_f_f
        Eigen::Vector3d roa_F_F = x_needle_tilde.row(active_idx).transpose();
    
        // Get tet id        
        int tet_id = collision_info.at(active_idx).tet_element_id;

        // Define quaternions indices
        int quat_idx = active_idx - 1;

        // Correct quaternion indices
        if (quat_idx < 0) { quat_idx = 0; }

        // Find global orientations 
        Eigen::Quaterniond qa = q_needle_tilde.at(quat_idx);
            
        // Get rotation matrix of element j wrt to F
        Eigen::Matrix3d rot_f_F = qa.toRotationMatrix();

        // Get particle indices
        size_t idx0_tissue = tissue_mesh_elements(tet_id, 0);
        size_t idx1_tissue = tissue_mesh_elements(tet_id, 1);
        size_t idx2_tissue = tissue_mesh_elements(tet_id, 2);
        size_t idx3_tissue = tissue_mesh_elements(tet_id, 3);

        // Get particles inertial positions
        Eigen::Vector3d xa_tissue = x_tissue_tilde.row(idx0_tissue).transpose();
        Eigen::Vector3d xb_tissue = x_tissue_tilde.row(idx1_tissue).transpose();
        Eigen::Vector3d xc_tissue = x_tissue_tilde.row(idx2_tissue).transpose();
        Eigen::Vector3d xd_tissue = x_tissue_tilde.row(idx3_tissue).transpose();

        // Vector rob_f_f
        Eigen::Vector3d rob_F_F = 0.25 * (xa_tissue + xb_tissue + xc_tissue +
            xd_tissue);

        // Define p vector
        Eigen::Vector3d p_vec = rob_F_F - roa_F_F;

        // Vector rab_f_f
        Eigen::Vector3d rab_f_f = rot_f_F.transpose() * p_vec;

        // Vector rab_f_f_0 or a
        Eigen::Vector3d rab_f0_f0 =
            initial_collision_array.row(active_idx).transpose();

        // Get matrix rot_f0_F
        Eigen::Matrix3d rot_f0_F = initial_collision_rot_array.at(active_idx);
        
        // Rotation matrix rot_f0_f
        Eigen::Matrix3d rot_f0_f = rot_f_F.transpose() * rot_f0_F;

        // Define vector s 
        Eigen::Vector3d s_vec = rot_f0_F * rab_f0_f0;

        // Define u vector
        Eigen::Vector3d u_vec = p_vec - s_vec;

        // Evaluate constraint
        Eigen::Vector3d ck = selection_matrix * rot_f_F.transpose() * u_vec;
        
        // Calculate gradients with respect to the tissue partices (same for all tissue particles)
        Eigen::Matrix3d dck_dxm_tissue = selection_matrix *
            rot_f_F.transpose() * 0.25;

        // Calculate gradients with respect to the needle particles
        Eigen::Matrix3d dck_dna = - selection_matrix * rot_f_F.transpose();

        // Calculate gradients with repsect to the needle rotation
        Eigen::MatrixXd w_mat = Eigen::MatrixXd::Zero(3, 4);
        w_mat.col(0) = 2.0 * qa.w() * u_vec + 2.0 * dme::S(u_vec) * qa.vec();
        w_mat.block(0, 1, 3, 3) = 2.0 * qa.vec().transpose() * u_vec *
            Eigen::Matrix3d::Identity() + 2.0 * qa.vec() * u_vec.transpose() - 
            2.0 * u_vec * qa.vec().transpose() + 2.0 * qa.w() * dme::S(u_vec);

        Eigen::MatrixXd dck_dqa = selection_matrix * w_mat;

        // Get inverse masses of associated tissue particles
        double wa_tissue = tissue_inv_masses(idx0_tissue);
        double wb_tissue = tissue_inv_masses(idx1_tissue);
        double wc_tissue = tissue_inv_masses(idx2_tissue);
        double wd_tissue = tissue_inv_masses(idx3_tissue);

        // Get inverse masses of associated needle particles
        double wp_needle = needle_inv_masses(active_idx);

        // Get inertia for bodies 0 
        Eigen::Matrix3d inertia0_body = needle_inertia_tensors.at(quat_idx);

        // Approximate inverse inertial
        double wq_needle = 1.0 / inertia0_body(1, 1);

        // Specify weighted sum of gradients
        Eigen::Matrix3d wsg =
            dck_dxm_tissue * wa_tissue * dck_dxm_tissue.transpose() + 
            dck_dxm_tissue * wb_tissue * dck_dxm_tissue.transpose() + 
            dck_dxm_tissue * wc_tissue * dck_dxm_tissue.transpose() + 
            dck_dxm_tissue * wd_tissue * dck_dxm_tissue.transpose() + 
            dck_dna * wp_needle * dck_dna.transpose() + 
            dck_dqa * wq_needle * dck_dqa.transpose();

        // Delta lagrange
        Eigen::Vector3d delta_lagrange = (wsg +
            alpha_tilde).colPivHouseholderQr().solve(- ck - alpha_tilde *
                lambda_lateral_nti.at(i));

        // Update lagrange
        lambda_lateral_nti.at(i) += delta_lagrange;

        // Update positions
        x_tissue_tilde.row(idx0_tissue) += (wa_tissue *
            dck_dxm_tissue.transpose() * delta_lagrange).transpose();

        x_tissue_tilde.row(idx1_tissue) += (wb_tissue *
            dck_dxm_tissue.transpose() * delta_lagrange).transpose();

        x_tissue_tilde.row(idx2_tissue) += (wc_tissue *
            dck_dxm_tissue.transpose() * delta_lagrange).transpose();

        x_tissue_tilde.row(idx3_tissue) += (wd_tissue *
            dck_dxm_tissue.transpose() * delta_lagrange).transpose();
    
        x_needle_tilde.row(active_idx) += (wp_needle * 
            dck_dna.transpose() * delta_lagrange).transpose();
    
        // Find corrections for element0
        Eigen::Vector4d corr_q0_vec = wq_needle * dck_dqa.transpose() * delta_lagrange;

        q_needle_tilde.at(quat_idx).coeffs() += Eigen::Quaterniond(corr_q0_vec(0),
            corr_q0_vec(1), corr_q0_vec(2), corr_q0_vec(3)).coeffs();
    }        
}

// Needle tissue contact constraint
void XPBDConstraints::needle_tissue_tip_contact_constraint(double dts,
    const Eigen::VectorXd& tissue_inv_masses,
    const Eigen::VectorXd& needle_inv_masses,
    double nt_contact_compliance,
    const std::vector<int>& active_constraints_indices,
    const std::vector<CollisionsInterface::NeedleCollisionInfo>& collision_info,
    int init_tet_contact_idx,
    Eigen::Vector4d init_natural_coordinates,
    const Eigen::MatrixXi& tissue_mesh_elements,
    Eigen::Vector3d& contact_normal,
    Eigen::MatrixXd& x_tissue_tilde,
    Eigen::MatrixXd& x_needle_tilde,
    double& lambda_tissue_contact,
    double& lambda_needle_contact)
{
    // Calculate alpha tilde
    double alpha_tilde = nt_contact_compliance / pow(dts, 2);

    // Active constaints indices
    int active_idx = active_constraints_indices.at(0);

    // Get particle indices
    size_t idx0_tissue = tissue_mesh_elements(init_tet_contact_idx, 0);
    size_t idx1_tissue = tissue_mesh_elements(init_tet_contact_idx, 1);
    size_t idx2_tissue = tissue_mesh_elements(init_tet_contact_idx, 2);
    size_t idx3_tissue = tissue_mesh_elements(init_tet_contact_idx, 3);

    // Get particles inertial positions
    Eigen::Vector3d xa_tissue = x_tissue_tilde.row(idx0_tissue).transpose();
    Eigen::Vector3d xb_tissue = x_tissue_tilde.row(idx1_tissue).transpose();
    Eigen::Vector3d xc_tissue = x_tissue_tilde.row(idx2_tissue).transpose();
    Eigen::Vector3d xd_tissue = x_tissue_tilde.row(idx3_tissue).transpose();

    // Get position of contact point
    Eigen::Vector3d xq = collision_info.at(active_idx).point_inertial_position;

    // Evalulate roq_F_F
    Eigen::Vector3d roq_F_F = init_natural_coordinates(0) * xa_tissue +
        init_natural_coordinates(1) * xb_tissue + 
        init_natural_coordinates(2) * xc_tissue +
        init_natural_coordinates(3) * xd_tissue;

    // Evaluate constraint
    double cs = contact_normal.transpose() * (xq - roq_F_F);
        
    // Calculate gradients with respect to the tissue partices
    Eigen::RowVector3d dcs_dxa_tissue = - contact_normal.transpose() *
        init_natural_coordinates(0) * Eigen::Matrix3d::Identity();
    
    Eigen::RowVector3d dcs_dxb_tissue = - contact_normal.transpose() *
        init_natural_coordinates(1) * Eigen::Matrix3d::Identity();
        
    Eigen::RowVector3d dcs_dxc_tissue = - contact_normal.transpose() *
        init_natural_coordinates(2) * Eigen::Matrix3d::Identity();

    Eigen::RowVector3d dcs_dxd_tissue = - contact_normal.transpose() *
        init_natural_coordinates(3) * Eigen::Matrix3d::Identity();

    // Calculate gradients with respect to the needle particles
    Eigen::RowVector3d dcs_dx_needle = contact_normal.transpose() *
        Eigen::Matrix3d::Identity();
        
    // Get inverse masses of associated tissue particles
    double wa_tissue = tissue_inv_masses(idx0_tissue);
    double wb_tissue = tissue_inv_masses(idx1_tissue);
    double wc_tissue = tissue_inv_masses(idx2_tissue);
    double wd_tissue = tissue_inv_masses(idx3_tissue);

    // Get inverse masses of associated needle particles
    double wp_needle = needle_inv_masses(active_idx);

    // Weighted sum of gradients
    double wsg = wa_tissue * dcs_dxa_tissue.squaredNorm() +
        wb_tissue * dcs_dxb_tissue.squaredNorm() + 
        wc_tissue * dcs_dxc_tissue.squaredNorm() +
        wd_tissue * dcs_dxd_tissue.squaredNorm() + 
        wp_needle * dcs_dx_needle.squaredNorm();
            
    // Lagrange denominator
    double lagrange_den = wsg + alpha_tilde;

    if (lagrange_den < 1.0e-6) 
        return;

    // Calculate delta lagrange
    double delta_lagrange = - (cs + alpha_tilde * lambda_tissue_contact) /
        lagrange_den;

    // Update lagrange
    lambda_tissue_contact += delta_lagrange;

    // Update positions
    x_tissue_tilde.row(idx0_tissue) += (wa_tissue *
        dcs_dxa_tissue.transpose() * delta_lagrange).transpose();

    x_tissue_tilde.row(idx1_tissue) += (wb_tissue *
        dcs_dxb_tissue.transpose() * delta_lagrange).transpose();

    x_tissue_tilde.row(idx2_tissue) += (wc_tissue *
        dcs_dxc_tissue.transpose() * delta_lagrange).transpose();

    x_tissue_tilde.row(idx3_tissue) += (wd_tissue *
        dcs_dxd_tissue.transpose() * delta_lagrange).transpose();
    
    x_needle_tilde.row(active_idx) += (wp_needle * 
        dcs_dx_needle.transpose() * delta_lagrange).transpose();
}

// Darboux vector as defined in (1)
Eigen::Vector3d XPBDConstraints::calculate_darboux_vector(const
    Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1, double element_length)
{
	return (2.0 / element_length) * (q0.conjugate() * q1).vec();
}


// Solve Saint Venant - Kirchhoff model
void XPBDConstraints::solve_st_venant_kirchhoff_constraints(double dts, 
    const Eigen::VectorXd& inv_masses,
    const std::vector<Eigen::Matrix3d>& inv_ref_shape_matrices,
    const Eigen::VectorXd& rest_volume,
    const std::vector<Eigen::Matrix<double, 6, 6>>& inv_elastic_coef_matrices,
    const Eigen::MatrixXi& tet_elements,
    Eigen::MatrixXd& x_tilde,
    std::vector<Eigen::Matrix<double, 6, 1>>& lambda_svk)
{
    // Identity matrix
    Eigen::Matrix3d i33_mat = Eigen::Matrix3d::Identity();
    
    // Loop through elements     
    for (size_t i = 0; i < tet_elements.rows(); i++)
    {
        // Get particle indices
        size_t idx0 = tet_elements(i, 0);
        size_t idx1 = tet_elements(i, 1);
        size_t idx2 = tet_elements(i, 2);
        size_t idx3 = tet_elements(i, 3);

        // Get inverse masses
        double inv_mass0 = inv_masses(idx0);
        double inv_mass1 = inv_masses(idx1);
        double inv_mass2 = inv_masses(idx2);
        double inv_mass3 = inv_masses(idx3);

        // Get particle coordinates
        Eigen::Vector3d x0 = x_tilde.row(idx0).transpose();
        Eigen::Vector3d x1 = x_tilde.row(idx1).transpose();
        Eigen::Vector3d x2 = x_tilde.row(idx2).transpose();
        Eigen::Vector3d x3 = x_tilde.row(idx3).transpose();
    
        // Convert tet vertices to a autodiff vector    
        Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1> x_vec = 
            Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1>::Zero(12, 1);
        x_vec << x0, x1, x2, x3;
        
        // Create function pointer
        Eigen::Matrix<autodiff::real, 6, 1>
            (*st_venant_kirchhoff_constraint_ptr)(const
            Eigen::Matrix<autodiff::real, Eigen::Dynamic, 1>&  x_vec,
            const Eigen::Matrix3d& ref_shape_matrix_inv);

        st_venant_kirchhoff_constraint_ptr =
            &XPBDConstraints::st_venant_kirchhoff_constraint;
    
        // Calculate constraint and its deriviative
        autodiff::VectorXreal cs_a;
        Eigen::MatrixXd dcs_dx = autodiff::jacobian(
            st_venant_kirchhoff_constraint_ptr,
            autodiff::wrt(x_vec),
            autodiff::at(x_vec, inv_ref_shape_matrices.at(i)), cs_a);
        
        // Cast result to double
        Eigen::VectorXd cs = cs_a.cast<double>();

        // Get gradients for each vector
        Eigen::MatrixXd dcs_dx0 =  dcs_dx.block(0, 0, 6, 3);
        Eigen::MatrixXd dcs_dx1 =  dcs_dx.block(0, 3, 6, 3);
        Eigen::MatrixXd dcs_dx2 =  dcs_dx.block(0, 6, 6, 3);
        Eigen::MatrixXd dcs_dx3 =  dcs_dx.block(0, 9, 6, 3);

        // Specify alpha_tilde matrix
        Eigen::Matrix<double, 6, 6> alpha_tilde = (1.0 / pow(dts, 2.0)) * 
            (1.0 / rest_volume(i)) * inv_elastic_coef_matrices.at(i);

	    // Specify weighted sum of gradients
        Eigen::Matrix<double, 6, 6> wsg =
            dcs_dx0 * inv_mass0 * dcs_dx0.transpose() + 
            dcs_dx1 * inv_mass1 * dcs_dx1.transpose() + 
            dcs_dx2 * inv_mass2 * dcs_dx2.transpose() + 
            dcs_dx3 * inv_mass3 * dcs_dx3.transpose();

        // Delta lagrange
	    Eigen::Matrix<double, 6, 1> delta_lagrange = (wsg +
            alpha_tilde).colPivHouseholderQr().solve(- cs - alpha_tilde *
            lambda_svk.at(i));

        // Update lagrange
        lambda_svk.at(i) += delta_lagrange;
    
	    // Find corrections
	    x_tilde.row(idx0) += (inv_mass0 * dcs_dx0.transpose() *
            delta_lagrange).transpose();
	    
	    x_tilde.row(idx1) += (inv_mass1 * dcs_dx1.transpose() *
            delta_lagrange).transpose();
    
	    x_tilde.row(idx2) += (inv_mass2 * dcs_dx2.transpose() *
            delta_lagrange).transpose();

	    x_tilde.row(idx3) += (inv_mass3 * dcs_dx3.transpose() *
            delta_lagrange).transpose();
    }
}

// St. Venant - Kirchoff constraint
template <class D>
Eigen::Matrix<D, 6, 1> XPBDConstraints::st_venant_kirchhoff_constraint(
    const Eigen::Matrix<D, Eigen::Dynamic, 1>&  x_vec,
    const Eigen::Matrix3d& ref_shape_matrix_inv)
{
    // Calculate ds matrix
    Eigen::Matrix<D, 3, 1> x1 = x_vec.block(0, 0, 3, 1);
    Eigen::Matrix<D, 3, 1> x2 = x_vec.block(3, 0, 3, 1);
    Eigen::Matrix<D, 3, 1> x3 = x_vec.block(6, 0, 3, 1);
    Eigen::Matrix<D, 3, 1> x4 = x_vec.block(9, 0, 3, 1);
     
    // Calculate the deformed shape matrix Ds
    Eigen::Matrix<D, 3, 3> ds_mat = Eigen::Matrix<D, 3, 3>::Zero();
    ds_mat.col(0) = x1 - x4;
    ds_mat.col(1) = x2 - x4;
    ds_mat.col(2) = x3 - x4;   

    // Calculate deformation gradient
    Eigen::Matrix<D, 3, 3> def_grad = ds_mat * ref_shape_matrix_inv;

    // Compute Green strain tensor
    Eigen::Matrix<D, 3, 3> green_strain = 0.5 * (def_grad.transpose() * 
        def_grad - Eigen::Matrix3d::Identity());

    // Strain in Voigt notation
    Eigen::Matrix<D, 6, 1> epsilon_voigt;
    epsilon_voigt << green_strain(0, 0), green_strain(1, 1), green_strain(2, 2),
        green_strain(0, 1), green_strain(0, 2), green_strain(1, 2);

    return epsilon_voigt;
}

// Solve Neo-Hookean hydrostatic constraints
void XPBDConstraints::solve_neo_hookean_hydrostatic_constraints(double dts,
    const Eigen::VectorXd& inv_masses,
    const std::vector<Eigen::Matrix3d>& inv_ref_shape_matrices,
    const Eigen::VectorXd& rest_volume,
    const Eigen::VectorXd& lame_mu,
    const Eigen::VectorXd& lame_lambda,
    const Eigen::MatrixXi& tet_elements,
    Eigen::MatrixXd& x_tilde,
    Eigen::VectorXd& lambda_nhh)
{
    // Identity matrix
    Eigen::Matrix3d i33_mat = Eigen::Matrix3d::Identity();
    
    #pragma omp parallel num_threads(4)
    {
        // Loop through elements     
        #pragma omp for
        for (size_t i = 0; i < tet_elements.rows(); i++)
        {
            // Get particle indices
            size_t idx0 = tet_elements(i, 0);
            size_t idx1 = tet_elements(i, 1);
            size_t idx2 = tet_elements(i, 2);
            size_t idx3 = tet_elements(i, 3);

            // Get inverse masses
            double inv_mass0 = inv_masses(idx0);
            double inv_mass1 = inv_masses(idx1);
            double inv_mass2 = inv_masses(idx2);
            double inv_mass3 = inv_masses(idx3);

            // Get particle coordinates
            Eigen::Vector3d x0 = x_tilde.row(idx0).transpose();
            Eigen::Vector3d x1 = x_tilde.row(idx1).transpose();
            Eigen::Vector3d x2 = x_tilde.row(idx2).transpose();
            Eigen::Vector3d x3 = x_tilde.row(idx3).transpose();

            // Calculate the deformed shape matrix Ds
            Eigen::Matrix3d ds_mat = Eigen::Matrix3d::Zero();
            ds_mat.col(0) = x0 - x3;
            ds_mat.col(1) = x1 - x3;
            ds_mat.col(2) = x2 - x3;   

            // Calculate deformation gradient
            Eigen::Matrix3d def_grad = ds_mat * inv_ref_shape_matrices.at(i);

            // Define deformation gradient components
            Eigen::Vector3d f0 = def_grad.col(0);
            Eigen::Vector3d f1 = def_grad.col(1);
            Eigen::Vector3d f2 = def_grad.col(2);

            // Calculate gamma
            double gamma = 1.0 + lame_mu(i) / lame_lambda(i);

            // Calculate hydrostatic constraint 
            double ch = def_grad.determinant() - gamma;

            // Calculate hydrostatic cosntraint gradient for the first three vertices
            Eigen::Matrix3d f_bar_mat = Eigen::Matrix3d::Zero();
            f_bar_mat.col(0) = f1.cross(f2);
            f_bar_mat.col(1) = f2.cross(f0);
            f_bar_mat.col(2) = f0.cross(f1);

            Eigen::Matrix3d dch_dx_bar = f_bar_mat *
                inv_ref_shape_matrices.at(i).transpose();

            // Get gradients
            Eigen::RowVector3d dch_dx0 = dch_dx_bar.col(0).transpose();
            Eigen::RowVector3d dch_dx1 = dch_dx_bar.col(1).transpose();
            Eigen::RowVector3d dch_dx2 = dch_dx_bar.col(2).transpose();
            Eigen::RowVector3d dch_dx3 = - (dch_dx0 + dch_dx1 + dch_dx2);
        
            // Weighted sum of gradients
            double wsg = inv_mass0 * dch_dx0.squaredNorm() +
                inv_mass1 * dch_dx1.squaredNorm() +
                inv_mass2 * dch_dx2.squaredNorm() +
                inv_mass3 * dch_dx3.squaredNorm();

            // Calculate apha
            double alpha = 1.0 / (rest_volume(i) * lame_lambda(i));

            // Calculate alpha tilde
            double alpha_tilde = alpha / pow(dts, 2.0);

            // Langrange denominator
            double lagrange_den = wsg + alpha_tilde;

            // if (lagrange_den < 1.e-7)
            //     return;

            // Calculate delta lagrange
            double delta_lagrange = - (ch + alpha_tilde * lambda_nhh(i)) / 
                lagrange_den;
        
            // Update positions
            x_tilde.row(idx0) += (inv_mass0 * dch_dx0 * delta_lagrange);
            x_tilde.row(idx1) += (inv_mass1 * dch_dx1 * delta_lagrange);
            x_tilde.row(idx2) += (inv_mass2 * dch_dx2 * delta_lagrange);
            x_tilde.row(idx3) += (inv_mass3 * dch_dx3 * delta_lagrange);

            // Update lagrange multipliners
            lambda_nhh(i) += delta_lagrange;
        }
    }
}

// Solve Neo-Hookean hydrostatic constraints
void XPBDConstraints::solve_neo_hookean_deviatoric_constraints(double dts,
    const Eigen::VectorXd& inv_masses,
    const std::vector<Eigen::Matrix3d>& inv_ref_shape_matrices,
    const Eigen::VectorXd& rest_volume,
    const Eigen::VectorXd& lame_mu,
    const Eigen::VectorXd& lame_lambda,
    const Eigen::MatrixXi& tet_elements,
    Eigen::MatrixXd& x_tilde,
    Eigen::VectorXd& lambda_nhd)
{
    // Loop through elements     
    #pragma omp parallel num_threads(4)
    {
        #pragma omp for
        for (size_t i = 0; i < tet_elements.rows(); i++)
        {
            // Get particle indices
            size_t idx0 = tet_elements(i, 0);
            size_t idx1 = tet_elements(i, 1);
            size_t idx2 = tet_elements(i, 2);
            size_t idx3 = tet_elements(i, 3);

            // Get inverse masses
            double inv_mass0 = inv_masses(idx0);
            double inv_mass1 = inv_masses(idx1);
            double inv_mass2 = inv_masses(idx2);
            double inv_mass3 = inv_masses(idx3);

            // Get particle coordinates
            Eigen::Vector3d x0 = x_tilde.row(idx0).transpose();
            Eigen::Vector3d x1 = x_tilde.row(idx1).transpose();
            Eigen::Vector3d x2 = x_tilde.row(idx2).transpose();
            Eigen::Vector3d x3 = x_tilde.row(idx3).transpose();

            // Calculate the deformed shape matrix Ds
            Eigen::Matrix3d ds_mat = Eigen::Matrix3d::Zero();
            ds_mat.col(0) = x0 - x3;
            ds_mat.col(1) = x1 - x3;
            ds_mat.col(2) = x2 - x3;   

            // Calculate deformation gradient
            Eigen::Matrix3d def_grad = ds_mat * inv_ref_shape_matrices.at(i);

            // Define deformation gradient components
            Eigen::Vector3d f0 = def_grad.col(0);
            Eigen::Vector3d f1 = def_grad.col(1);
            Eigen::Vector3d f2 = def_grad.col(2);

            // Calculate constaint
            double cd_bar = f0.squaredNorm() + f1.squaredNorm() +
                f2.squaredNorm();
            double cd = sqrt(cd_bar);

            // Calculate deviatoric constraint gradient for the first three vertices
            Eigen::Matrix3d dcd_dx_bar = (1.0 / cd) * def_grad *
                inv_ref_shape_matrices.at(i).transpose();

            // Get gradients
            Eigen::RowVector3d dcd_dx0 = dcd_dx_bar.col(0).transpose();
            Eigen::RowVector3d dcd_dx1 = dcd_dx_bar.col(1).transpose();
            Eigen::RowVector3d dcd_dx2 = dcd_dx_bar.col(2).transpose();
            Eigen::RowVector3d dcd_dx3 = - (dcd_dx0 + dcd_dx1 + dcd_dx2);
        
            // Weighted sum of gradients
            double wsg = inv_mass0 * dcd_dx0.squaredNorm() +
                inv_mass1 * dcd_dx1.squaredNorm() +
                inv_mass2 * dcd_dx2.squaredNorm() +
                inv_mass3 * dcd_dx3.squaredNorm();

            // Calculate apha
            double alpha = 1.0 / (rest_volume(i) * lame_mu(i));

            // Calculate alpha tilde
            double alpha_tilde = alpha / pow(dts, 2.0);

            // Langrange denominator
            double lagrange_den = wsg + alpha_tilde;

            // if (lagrange_den < 1.e-6)
            //     return;

            // Calculate delta lagrange
            double delta_lagrange = - (cd + alpha_tilde * lambda_nhd(i)) / 
                lagrange_den;
        
            // Update positions
            x_tilde.row(idx0) += (inv_mass0 * dcd_dx0 * delta_lagrange);
            x_tilde.row(idx1) += (inv_mass1 * dcd_dx1 * delta_lagrange);
            x_tilde.row(idx2) += (inv_mass2 * dcd_dx2 * delta_lagrange);
            x_tilde.row(idx3) += (inv_mass3 * dcd_dx3 * delta_lagrange);

            // Update lagrange multipliners
            lambda_nhd(i) += delta_lagrange;
        }
    }
}


// Extract tetrahedron natural coordinates
Eigen::Vector4d XPBDConstraints::extract_linear_tet_natural_coordinates(
    const Eigen::Vector3d& x, const Eigen::Vector3d& xa,
    const Eigen::Vector3d& xb, const Eigen::Vector3d& xc,
    const Eigen::Vector3d& xd)
{
    // Find mapping matrix
    Eigen::Matrix3d j_mat = Eigen::Matrix3d::Zero();
    j_mat.col(0) = xa - xd;
    j_mat.col(1) = xb - xd;
    j_mat.col(2) = xc - xd;

    // Find the first three natural coordinates
    Eigen::Vector3d phi_vec3 = j_mat.colPivHouseholderQr().solve(x-xd);

    // Find the fourth natural coordinate
    double phi_4 = 1.0 - phi_vec3(0) - phi_vec3(1) - phi_vec3(2);

    return Eigen::Vector4d(phi_vec3(0), phi_vec3(1), phi_vec3(2), phi_4);
}

// Explicit template initialization 
using NScalar = double;
using AScalar = autodiff::real;

template Eigen::Matrix<NScalar, 6, 1>
    XPBDConstraints::st_venant_kirchhoff_constraint<NScalar>(
        const Eigen::Matrix<NScalar, Eigen::Dynamic, 1>&,
        const Eigen::Matrix3d&);

template Eigen::Matrix<AScalar, 6, 1>
    XPBDConstraints::st_venant_kirchhoff_constraint<AScalar>(
        const Eigen::Matrix<AScalar, Eigen::Dynamic, 1>&,
        const Eigen::Matrix3d&);
