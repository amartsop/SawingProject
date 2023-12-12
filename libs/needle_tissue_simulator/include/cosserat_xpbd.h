#pragma once 

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dynamics_math_eigen/dynamics_math_eigen.h>

/**
 * @brief This class is an implementation of
 *  "Position And Orientation Based Cosserat Rods" paper (1)
 * (https://animation.rwth-aachen.de/publication/0550/)
 */
	
class CosseratXPBD
{
public:
    CosseratXPBD(){};

    /** Determine the position and orientation corrections for the stretch and shear constraint constraint (eq. 37 in the paper). \n\n
    *
    * @param x0 position of first particle
    * @param inv_mass0 inverse mass of first particle
    * @param x1 position of second particle
    * @param inv_mass1 inverse mass of second particle
    * @param q0 Quaternionr at the center of the edge
    * @param inv_maass_q0 inverse mass of the quaternion
    * @param k_strethcing_shear stiffness coefficients for stretching and shearing
    * @param rest_length rest edge length
    * @param corr_x0 position correction of first particle
    * @param corr_x1 position correction of second particle
    * @param corr_q0 orientation correction of quaternion
    */
    
    static bool solve_stretch_shear_constraint(
        const Eigen::Vector3d& x0, double inv_mass0,
        const Eigen::Vector3d& x1, double inv_mass1,
        const Eigen::Quaterniond& q0,
        const Eigen::Matrix3d& inertia0_body,
        const Eigen::Vector3d& lambda_stretch,
        const Eigen::Matrix3d& a_tilde_stretch,
        const double element_length,
        Eigen::Vector3d& corr_x0,
        Eigen::Vector3d& corr_x1,
        Eigen::Quaterniond& corr_q0,
        Eigen::Vector3d& corr_lambda_stretch);

    /** Determine the position corrections for the bending and torsion constraint constraint (eq. 40 in the paper). \n\n
    *
    * @param q0 first quaternion
    * @param inv_mass_q0 inverse mass of the first quaternion
    * @param q1 second quaternion
    * @param inv_mass_q1 inverse Mass of the second quaternion
    * @param k_bending_torsion stiffness coefficients for stretching and shearing
    * @param rest_darboux_vector rest Darboux vector
    * @param corr_q0 position correction of first particle
    * @param corr__q1 position correction of second particle
    */
    static bool solve_bend_twist_constraint(
        const Eigen::Quaterniond& q0, const Eigen::Matrix3d& inertia0_body,
        const Eigen::Quaterniond& q1, const Eigen::Matrix3d& inertia1_body,
        const Eigen::Vector3d& lambda_bend, const double element_length,
        const Eigen::Matrix3d& a_tilde_bend,
        const Eigen::Vector3d& rest_darboux_vector,
        Eigen::Quaterniond& corr_q0, Eigen::Quaterniond&  corr_q1, 
        Eigen::Vector3d& corr_lambda_bend);

    /*
        G matrix quaternion, see Shabana - Dynamics of multibody systems (2005), 
        eq. 3.109
    */ 
    static void calculate_g_matrix(const Eigen::Quaterniond& q,
        Eigen::Matrix<double, 3, 4>& g_mat);

    /*
        Darboux vector as defined in (1)
    */ 
    static Eigen::Vector3d calculate_darboux_vector(const Eigen::Quaterniond& 
        q0, const Eigen::Quaterniond& q1, double element_length);

};