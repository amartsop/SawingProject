#pragma once 

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dynamics_math_eigen/dynamics_math_eigen.h>

class NeedleBoundaryInterface
{
public:
    NeedleBoundaryInterface(size_t particles_num, size_t elements_num, 
        const Eigen::Vector3d& roa_F_F, const Eigen::Vector3d& euler_f);

    void get_initial_pose(Eigen::Vector3d& roa_F_F_0,
        Eigen::Quaterniond& quat0) {
        roa_F_F_0 = m_roa_F_F_0; quat0 = m_quat0;
    }

    // Get boundary points positions
    void get_desired_points_positions(Eigen::MatrixXd& dpp,
        Eigen::VectorXi& bpi)
    {
        dpp = m_desired_points_positions; bpi = m_boundary_points_indices;
    }

    // Get boundary points positions
    void get_desired_quaternions(std::vector<Eigen::Quaterniond>& dq, 
        Eigen::VectorXi& bqi)
    {
        dq = m_desired_quaternions; bqi = m_boundary_points_indices;
    }

    // Update 
    void update(double real_time);

private:

    // Number of particles
    size_t m_np;

    // Desired point position
    Eigen::MatrixXd m_desired_points_positions;
    
    // Boundary points indices
    Eigen::VectorXi m_boundary_points_indices;

private:

    // Desired quaternions
    std::vector<Eigen::Quaterniond> m_desired_quaternions;

    // Boundary quaternions indices
    Eigen::VectorXi m_boundary_quat_indices;

    // Number of elements/quaternions
    size_t m_nq;

private:

    // Initial base position
    Eigen::Vector3d m_roa_F_F_0;

    // Initial base orientation
    Eigen::Quaterniond m_quat0;

    // Initial handle position
    Eigen::Vector3d m_roc_F_F_0;

    // Distance between rigid body's com and reference frame a 
    Eigen::Vector3d m_rca_f_f;

private:

    // Update handle pose 
    void update_handle_pose(double t, Eigen::Vector3d& roc_F_F,
        Eigen::Quaterniond& quat_f);
};