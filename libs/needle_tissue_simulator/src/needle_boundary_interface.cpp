#include "../include/needle_boundary_interface.h"

NeedleBoundaryInterface::NeedleBoundaryInterface(size_t particles_num,
    size_t elements_num, const Eigen::Vector3d& roa_F_F, 
    const Eigen::Vector3d& euler_f)
{
    // Set particles num
    m_np = particles_num;
    
    // Set elements num
    m_nq = elements_num;
    
    // Number of driving points
    int dp_num = 1;

    // Setup driving point
    m_desired_points_positions = Eigen::MatrixXd::Zero(m_np, 3);
    
    // Set position of driving point
    m_desired_points_positions.row(0) = m_roa_F_F_0.transpose();

    // Initialize boundary points indices 
    m_boundary_points_indices = Eigen::VectorXi::Zero(dp_num, 1);

    // Initialize desired quaternions
    m_desired_quaternions = std::vector<Eigen::Quaterniond>(m_nq,
        Eigen::Quaterniond::Identity());

    // Initialize boundary quaternions indices (only first quaternion with index 0)
    m_boundary_quat_indices = Eigen::VectorXi::Zero(dp_num, 1);
}

// Update
void NeedleBoundaryInterface::update(double real_time)
{
    // Get handle pose     
    Eigen::Vector3d roc_F_F; Eigen::Quaterniond quat_f;
    update_handle_pose(real_time, roc_F_F, quat_f);
    
    // Get rotation matrix
    Eigen::Matrix3d rot_F_F = quat_f.toRotationMatrix();
    
    // Update base pose
    Eigen::Vector3d roa_F_F = roc_F_F + rot_F_F * m_rca_f_f;

    // Set boundaries
    m_desired_points_positions.row(0) = roa_F_F.transpose();
    m_desired_quaternions.at(0) = quat_f;
}


// Update handle pose 
void NeedleBoundaryInterface::update_handle_pose(double t,
    Eigen::Vector3d& roc_F_F, Eigen::Quaterniond& quat_f)
{
    /******************* Position *******************/
    // Amplitude (m)
    double a = 0.05;

    // Position amplitude (m)
    Eigen::Vector3d a_p = Eigen::Vector3d(0.0, 0.0, 0.0);

    // Position frequency (Hz)
    Eigen::Vector3d f_p = Eigen::Vector3d(0.5, 0.0, 0.5);

    // Position phase (rad)
    Eigen::Vector3d phi_p = Eigen::Vector3d(0.0, 0.0, 0.0);
    
    /******************* Orientation *******************/
    // Euler angles amplitude (rad)
    Eigen::Vector3d a_o = Eigen::Vector3d(0.0, 0.06, 0.0);

    // Euler angles frequency (Hz)
    Eigen::Vector3d f_o = Eigen::Vector3d(0.5, 0.2, 0.5);

    // Euler angles phase (rad)
    Eigen::Vector3d phi_o = Eigen::Vector3d(0.0, 0.0, 0.0);

    // Initialize euler angles
    Eigen::Vector3d theta = Eigen::Vector3d::Zero();

    /******************* Trajectory functions *******************/
    for (int i = 0; i < 3; ++i)
    {
        // Rigid body translational trajectory
        double a_dot = 2.0 * M_PI * f_p(i);
        double a = a_dot * t + phi_p(i);

        roc_F_F(i) = a_p(i) * sin(a) + m_roc_F_F_0(i);

        // Rigid body rotational trajectory
        double b_dot = 2.0 * M_PI * f_o(i);
        double b = b_dot * t + phi_o(i);
    
        theta(i) = a_o(i) * sin(b);
    }

    // Set quaternions
    quat_f = dme::EulerRotations::euler_to_quaternions(theta(0), theta(1), theta(2));
}
