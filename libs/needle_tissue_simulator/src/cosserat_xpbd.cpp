#include "../include/cosserat_xpbd.h"

bool CosseratXPBD::solve_stretch_shear_constraint(
	const Eigen::Vector3d& x0, double inv_mass0,
	const Eigen::Vector3d& x1, double inv_mass1,
	const Eigen::Quaterniond& q0,
	const Eigen::Matrix3d& inertia0_body,
	const Eigen::Vector3d& lambda_stretch,
	const Eigen::Matrix3d& a_tilde_stretch,
	const double element_length,
	Eigen::Vector3d& corr_x0, Eigen::Vector3d& corr_x1,
	Eigen::Quaterniond& corr_q0, Eigen::Vector3d& corr_lambda_stretch)
{
    // Initialize corrections
    corr_x0.setZero(); corr_x1.setZero();
    corr_q0.coeffs().setZero(); corr_lambda_stretch.setZero();

	// Epsilon 
	double eps = 1.e-6;

	// Define e1 as quaternion
	Eigen::Quaterniond e1_quat = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);

	// Inverse element length
	double inv_element_length = 1.0 / element_length;

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
		dcs_dq0 * w_in * dcs_dq0.transpose() + 
		eps * Eigen::Matrix3d::Identity();

	// Delta lambda
	corr_lambda_stretch = (wsg + a_tilde_stretch).ldlt().solve(- cs -
		a_tilde_stretch * lambda_stretch);

	// Find corrections
	corr_x0 = inv_mass0 * dcs_dx0.transpose() * corr_lambda_stretch;
	
	// Find corrections
	corr_x1 = inv_mass1 * dcs_dx1.transpose() * corr_lambda_stretch;

	// Find corrections
	Eigen::Vector4d corr_q0_vec = w_in * dcs_dq0.transpose() *
		corr_lambda_stretch;

	corr_q0 = Eigen::Quaterniond(corr_q0_vec(0), corr_q0_vec(1), 
		corr_q0_vec(2), corr_q0_vec(3));

	return true;
}

bool CosseratXPBD::solve_bend_twist_constraint(
	const Eigen::Quaterniond& q0, const Eigen::Matrix3d& inertia0_body,
	const Eigen::Quaterniond& q1, const Eigen::Matrix3d& inertia1_body,
	const Eigen::Vector3d& lambda_bend, const double element_length,
	const Eigen::Matrix3d& a_tilde_bend,
	const Eigen::Vector3d& rest_darboux_vector,
	Eigen::Quaterniond& corr_q0, Eigen::Quaterniond&  corr_q1, 
	Eigen::Vector3d& corr_lambda_bend)
{
    // Initialize corrections
    corr_q0.coeffs().setZero(); corr_q1.coeffs().setZero();
	corr_lambda_bend.setZero();

	// Epsilon	
	double eps = 1.0-6;

	// Calculate dsarboux vector
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

	// Approximate inverse inertial
	double w_in0 = 1.0 / inertia0_body(1, 1);
	double w_in1 = 1.0 / inertia1_body(1, 1);

	// Specify weighted sum of gradients
	Eigen::Matrix3d wsg = dcb_dq0 * w_in0 * dcb_dq0.transpose() + 
		dcb_dq1 * w_in1 * dcb_dq1.transpose() + 
		eps * Eigen::Matrix3d::Identity();

	// ssolve for delta lambda
	corr_lambda_bend = (wsg + a_tilde_bend).colPivHouseholderQr().solve(- cb -
		a_tilde_bend * lambda_bend);

	// Find corrections
	Eigen::Vector4d corr_q0_vec = w_in0 * dcb_dq0.transpose() * corr_lambda_bend;

	corr_q0 = Eigen::Quaterniond(corr_q0_vec(0), corr_q0_vec(1), corr_q0_vec(2),
		corr_q0_vec(3));
	
	// Find corrections
	Eigen::Vector4d corr_q1_vec = w_in1 * dcb_dq1.transpose() * corr_lambda_bend;

	corr_q1 = Eigen::Quaterniond(corr_q1_vec(0), corr_q1_vec(1), corr_q1_vec(2),
		corr_q1_vec(3));

	return true;
}

/*
	G matrix quaternion, see Shabana - Dynamics of multibody systems (2005), 
	eq. 3.109
*/ 
void CosseratXPBD::calculate_g_matrix(const Eigen::Quaterniond& q,
	Eigen::Matrix<double, 3, 4>& g_mat)
{
	// Set matrix to zero
	g_mat.setZero();

	// Store quaternion coeffs
	double q0 = q.w(); double q1 = q.x(); double q2 = q.y(); double q3 = q.z();

	// Define matrix
	g_mat << -q1, q0, q3, -q2, -q2, -q3, q0, q1, -q3, q2, -q1, q0;
	g_mat *= 2.0;
}

/*
	Darboux vector as defined in (1)
*/ 
Eigen::Vector3d CosseratXPBD::calculate_darboux_vector(const Eigen::Quaterniond& 
	q0, const Eigen::Quaterniond& q1, double element_length)
{
	return (2.0 / element_length) * (q0.conjugate() * q1).vec();
}