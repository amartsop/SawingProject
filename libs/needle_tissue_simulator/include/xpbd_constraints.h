#pragma once 

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dynamics_math_eigen/dynamics_math_eigen.h>
#include "collisions_interface.h"
#include <omp.h>

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

class XPBDConstraints
{
public:
    XPBDConstraints() {};

    // Solve edge contraints
    static void solve_edge_constraints(double dts,
        const Eigen::VectorXd& inv_masses,
        const Eigen::VectorXd& edge_lengths, 
        double edge_compliance, 
        const Eigen::MatrixXi& edges_ids,
        Eigen::MatrixXd& x_tilde,
        Eigen::VectorXd& lambda_edge);


    // Solve volume constraints
    static void solve_tet_volume_constraints(double dts,
        const Eigen::VectorXd& inv_masses,
        const Eigen::VectorXd& rest_volume,
        double volume_compliance,
        const Eigen::MatrixXi& tet_elements,
        Eigen::MatrixXd& x_tilde,
        Eigen::VectorXd& lambda_volume);

    // Solve boundary constraints
    static void solve_point_boundary_constraints_single(double dts, 
        const Eigen::VectorXd& inv_masses, 
        const Eigen::MatrixXd& desired_vertices_positions,
        double boundary_compliance,
        const Eigen::VectorXi& boundary_points_indices,
        Eigen::MatrixXd& x_tilde,
        Eigen::VectorXd& lambda_boundary);

    static void solve_point_boundary_constraints(double dts, 
        const Eigen::VectorXd& inv_masses,
        const Eigen::Matrix3d& boundary_compliance,
        const Eigen::MatrixXd& desired_positions,
        const Eigen::VectorXi& boundary_points_indices,
        Eigen::MatrixXd& x_tilde,
        std::vector<Eigen::Vector3d>& lambda_boundary);


    // Solve quaternion boundary constraints
    static void solve_quaternion_boundary_constraints(double dts, 
        const std::vector<Eigen::Matrix3d>& inertia_tensors,
        const std::vector<Eigen::Quaterniond>& desired_quaternions,
        const Eigen::Matrix3d& boundary_compliance,
        const Eigen::VectorXi& boundary_quaternion_indices,
        std::vector<Eigen::Quaterniond>& q_tilde,
        std::vector<Eigen::Vector3d>& lambda_bound);

public:
    
    // Solve St. Venant - Kirchhoff model
    static void solve_st_venant_kirchhoff_constraints(double dts, 
        const Eigen::VectorXd& inv_masses,
        const std::vector<Eigen::Matrix3d>& inv_ref_shape_matrices,
        const Eigen::VectorXd& rest_volume,
        const std::vector<Eigen::Matrix<double, 6, 6>>& inv_elastic_coef_matrices,
        const Eigen::MatrixXi& tet_elements,
        Eigen::MatrixXd& x_tilde,
        std::vector<Eigen::Matrix<double, 6, 1>>& lambda_svk);

    
    // St. Venant - Kirchhoff constraint
    template <class D>
    static Eigen::Matrix<D, 6, 1> st_venant_kirchhoff_constraint(const
        Eigen::Matrix<D, Eigen::Dynamic, 1>&  x_vec, const Eigen::Matrix3d& 
        ref_shape_matrix_inv);
        
    // Solve Neo-Hookean hydrostatic constraints
    static void solve_neo_hookean_hydrostatic_constraints(double dts,
        const Eigen::VectorXd& inv_masses,
        const std::vector<Eigen::Matrix3d>& inv_ref_shape_matrices,
        const Eigen::VectorXd& rest_volume,
        const Eigen::VectorXd& lame_mu,
        const Eigen::VectorXd& lame_lambda,
        const Eigen::MatrixXi& tet_elements,
        Eigen::MatrixXd& x_tilde,
        Eigen::VectorXd& lambda_nhh);

    // Solve Neo-Hookean deviatoric constraints
    static void solve_neo_hookean_deviatoric_constraints(double dts,
        const Eigen::VectorXd& inv_masses,
        const std::vector<Eigen::Matrix3d>& inv_ref_shape_matrices,
        const Eigen::VectorXd& rest_volume,
        const Eigen::VectorXd& lame_mu,
        const Eigen::VectorXd& lame_lambda,
        const Eigen::MatrixXi& tet_elements,
        Eigen::MatrixXd& x_tilde,
        Eigen::VectorXd& lambda_nhd);

public:

    /** Determine the position and orientation corrections for the
    * stretch and shear cosserat constraints.
    */
    static void cosserat_stretch_shear_constraints(
        double dts, const Eigen::VectorXd& inv_masses,
        const std::vector<Eigen::Matrix3d>& inertia_tensors,
        double element_length,
        const Eigen::Matrix3d& stretch_compliance, 
        const Eigen::MatrixXd& pcp,
        Eigen::MatrixXd& x_tilde,
        std::vector<Eigen::Quaterniond>& q_tilde,
        std::vector<Eigen::Vector3d>& lambda_stretch);

    /** Determine the position corrections for the bending and torsion
     * constraint constraint (eq. 40 in the paper).
    */
    static void cosserat_bend_twist_constraints(double dts,
        const std::vector<Eigen::Matrix3d>& inertia_tensors,
        const double element_length,
        const Eigen::Matrix3d& bending_compliance, 
        const Eigen::Vector3d& rest_darboux_vector,
        const Eigen::MatrixXd& ecp,
        std::vector<Eigen::Quaterniond>& q_tilde,
        std::vector<Eigen::Vector3d>& lambda_bend);

    // Needle tissue interaction lateral constraints
    static void needle_tissue_interaction_lateral_constraints(double dts, 
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
        Eigen::VectorXd& lambda_lateral_nti);

    // Needle tissue interaction lateral constraints 3d dimensions
    static void needle_tissue_interaction_lateral_constraints_3d(double dts, 
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
        std::vector<Eigen::Vector3d>& lambda_lateral_nti);

    // Needle tissue tip contact constraint
    static void needle_tissue_tip_contact_constraint(double dts,
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
        double& lambda_needle_contact);


public:

    /**
     * @brief Extract the natural coordinates of a linear tetrahedron given the 
     * inertial position of a point inside it and the inertial positions of its 
     * four nodes.
     * @param x The inertial position of the point for which we want to find the 
     * natural coordinates.
     * @param xa The inertial position of the tetrahedron's node A.
     * @param xb The inertial position of the tetrahedron's node B.
     * @param xc The inertial position of the tetrahedron's node C.
     * @param xd The inertial position of the tetrahedron's node D.
     * @return Eigen::Vector4d The 4d vector of the point's natural coordinats
     * [phi_a, phi_b, phi_c, phi_d]^T
     */
    static Eigen::Vector4d extract_linear_tet_natural_coordinates(
        const Eigen::Vector3d& x, const Eigen::Vector3d& xa,
        const Eigen::Vector3d& xb, const Eigen::Vector3d& xc,
        const Eigen::Vector3d& xd);

public:

    // Calculate tetrahedron volume
    static double tetrahedron_volume(const Eigen::Vector3d& p0,
        const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
        const Eigen::Vector3d& p3);
    
    // Darboux vector as defined in (1)
    static Eigen::Vector3d calculate_darboux_vector(const Eigen::Quaterniond& 
        q0, const Eigen::Quaterniond& q1, double element_length);

};