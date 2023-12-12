#pragma once 

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dynamics_math_eigen/dynamics_math_eigen.h>


#include "needle_model.h"
#include "physics_params.hpp"
#include "xpbd_constraints.h"

class NeedleModel
{
public:
    NeedleModel(const Eigen::Vector3d& roa_F_F, const Eigen::Vector3d& euler_f,
        PhysicsParams* physics_params, int needle_elements=20);

    // Update needle model model
    void predict_solution(double real_time, double dt,
        const Eigen::MatrixXd& fext, const Eigen::MatrixXd& text);

    // Apply constraints 
    void constrain_iteration(double real_time, double dt);

    // Correct solution
    void update_solution(double real_time, double dt);

public:

    // Get elements number 
    int get_number_of_elements(void) { return m_ne; }
    
    // Get number of particles
    int get_number_of_particles(void) { return m_np; }
    
    // Get number of quaternions
    int get_number_of_quaternions(void) { return m_nq; }

    // Get particles masses
    Eigen::VectorXd get_particles_masses(void) { return m_masses; }

    // Get particles inverse masses 
    Eigen::VectorXd get_particles_inverse_masses(void) { return m_inv_masses; }

    // Get elements inertial tensors
    std::vector<Eigen::Matrix3d> get_elements_inertial_tensors(void) {
        return m_inertia_tensors;
    }

    // Get needle's bevel tip
    double get_needle_bevel_tip_angle(void) { return m_bevel_tip_angle; }

    // Get needle reaction force
    void get_needle_generalized_reaction_force(Eigen::Vector3d& force, 
        Eigen::Vector3d& moment)
    {
        force = m_reaction_force; moment = m_reaction_moment;
    }

public:
    
    // Get needle state
    void get_state(Eigen::MatrixXd& particles_positions,
        std::vector<Eigen::Quaterniond>& elements_rotation)
    {
        particles_positions = m_pos; elements_rotation = m_quat;
    }

    // Get needle velocities
    void get_velocities(Eigen::MatrixXd& particles_velocities,
        Eigen::MatrixXd& elements_rot_velocities)
    {
        particles_velocities = m_vel; elements_rot_velocities = m_w;
    }

    // Get needle visual vertices
    void get_inertial_vertices(const Eigen::Vector3d& rap_f_f_0,  
        Eigen::Vector3d& rop_F_F);

    // Set base pose
    void set_base_pose(const Eigen::Vector3d& roa_F_F,
        const Eigen::Vector3d& euler_f);
    
    // Get base pose 
    void get_base_pose(Eigen::Vector3d& roa_F_F, Eigen::Vector3d& eul)
    {
        roa_F_F = m_roa_F_F;
        eul = dme::EulerRotations::quaternions_to_euler(m_quat_base);
    }

public:

    // Get needle position prediction pointer
    Eigen::MatrixXd* get_position_prediction_ptr(void) {
        return &m_x_tilde;
    }

    // Get needle rotation prediction pointer
    std::vector<Eigen::Quaterniond>* get_rotation_prediction_ptr(void) {
        return &m_q_tilde;
    }

private:
    
    // Number of elements (only even numbers)
    int m_ne;

    // Number of particles and number of quaternions
    int m_np, m_nq;

private:

    // Current pos (n x 3) (m)
    Eigen::MatrixXd m_pos;
    
    // Current Velocity (n x 3) (m/s)
    Eigen::MatrixXd m_vel;

    // Previous Velocity (n x 3) (m/s)
    Eigen::MatrixXd m_vel_prev;

    // Current rotation velocity wrt to body frame (rad/sec)
    Eigen::MatrixXd m_w;

    // Current quaterions
    std::vector<Eigen::Quaterniond> m_quat;

    // Initial position
    Eigen::MatrixXd m_pos0;

    // Length per element
    double m_element_length = 0.0;
    
private:

    // Define inertial properties 
    void define_inertial_properties(void);

    // Needle mass
    double m_needle_mass;

    // Mass per particle
    Eigen::VectorXd m_masses;

    // Inverse mass per particle
    Eigen::VectorXd m_inv_masses;

    // Inertial tensor per element
    std::vector<Eigen::Matrix3d> m_inertia_tensors;

    // Inverse inertial tensor per element (local inertia)
    std::vector<Eigen::Matrix3d> m_inv_inertia_tensors;
    
private:
    
    // Particles connection pairs number
    int m_pcp_num;

    // Particles connection pairs. Each row contains the connections between 
    // neighbouring elements (0-1, 1-2, 2-3, ....)
    Eigen::MatrixXd m_pcp;

    // Elements connection pairs number
    int m_ecp_num;

    // Elements conneection pairs 
    Eigen::MatrixXd m_ecp;
    
private:

    // XPBD prediction for particles positions
    Eigen::MatrixXd m_x_tilde;

    // XPBD prediction for elements rotations
    std::vector<Eigen::Quaterniond> m_q_tilde;
    
private:

    // Rest darboux vector 
    Eigen::Vector3d m_rest_darboux_vector;

    // Zero quaternion
    Eigen::Quaterniond m_zero_quat;

private:

    // Physics params parameter
    PhysicsParams* m_physics_params;

private:

    // Stretch compliance matrix
    Eigen::Matrix3d m_stretch_compliance_matrix;

    // Lagrange multipliers for cosserat stretch/shear
    std::vector<Eigen::Vector3d> m_lambda_stretch_shear;

    // Lagrange multipliers for cosserat bending/torsion
    std::vector<Eigen::Vector3d> m_lambda_bending_torsion;

    // Initialize cosserat contraints
    void initialize_cosserat_constraints(void);

    // Generate cosserat compliance matrices
    void generate_cosserat_compliance_matrices(void);

    // Bending compliance matrix
    Eigen::Matrix3d m_bend_compliance_matrix;

    // Translational damping matrix
    Eigen::Matrix3d m_trans_damping_matrix;

    // Bending damping matrix
    Eigen::Matrix3d m_bending_damping_matrix;
    
private:
    
    // Initialize boundary constraints
    void initialize_boundary_conditions(const Eigen::Vector3d& roa_F_F_0, 
        const Eigen::Quaterniond& quat0);

    /************** Position Constraints **********/

    // Base position
    Eigen::Vector3d m_roa_F_F;

    // Desired point position
    Eigen::MatrixXd m_desired_points_positions;
    
    // Boundary points indices
    Eigen::VectorXi m_boundary_points_indices;

    // Boundary compliance 
    Eigen::Matrix3d m_boundary_points_compliance;

    // Lagrange multipliers for position boundary
    std::vector<Eigen::Vector3d> m_lambda_position_bound;

private:

    /************** Rotation Constraints **********/

    // Base orientation
    Eigen::Quaterniond m_quat_base;

    // Desired quaternions
    std::vector<Eigen::Quaterniond> m_desired_quaternions;

    // Boundary quaternions indices
    Eigen::VectorXi m_boundary_quat_indices;

    // Boundary compliance for quaternion
    Eigen::Matrix3d m_boundary_quat_compliance;

    // Lagrange multipliers for quaternion boundary
    std::vector<Eigen::Vector3d> m_lambda_quat_bound;   

private:
    // Cosserat state 
    struct CosseratState
    {
        // Inertial position
        std::vector<Eigen::Vector3d> particles_pos;
    
        // Inertial rotation
        std::vector<Eigen::Matrix3d> quaternions_rot;

        // Rotation position 
        std::vector<Eigen::Vector3d> quaternions_pos;
    };
    
    // Update cosserat state
    void update_cosserat_state(void);
    
    // Cosserat state
    CosseratState m_cosserat_state;

    // Initial cosserat state
    CosseratState m_init_cosserat_state;

    // Cumulated length per element
    std::vector<double> m_cel;
    
private:

    // Search sorted algorithm
    int search_sorted_double(const std::vector<double>& array,
        double element);

    // Get initial state
    void get_initial_state(int elem_idx, Eigen::Vector3d& roa_F_F_0,
        Eigen::Matrix3d& rot_f_F_0, Eigen::Vector3d& raaj_f_f_0, 
        Eigen::Matrix3d& rot_fj_f_0);

private:

    // Calculate reaction forces
    void calculate_reaction_forces(double dt);

    // Reaction forces coeff
    double m_reaction_force_coeff = 1.e8;

    // Reaction moment coeff 
    double m_reaction_moment_coeff =  7.0e-7;

    // Reaction forces and moments
    Eigen::Vector3d m_reaction_force, m_reaction_moment;

private:

    // Needle bevel tip angle
    double m_bevel_tip_angle = 20 * M_PI / 180.;
};