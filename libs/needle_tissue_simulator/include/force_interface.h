#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "./physics_params.hpp"
#include "collisions_interface.h"
#include "friction_model.h"
#include "utils.h"

class ForceInterface
{
public:
    ForceInterface(PhysicsParams* physics_params);

    // Initialize tissue properties
    void initialize_tissue_properties(size_t tissue_particles_num,
        const Eigen::VectorXd& tissue_system_masses,
        const Eigen::MatrixXi& tissue_tet_elements);

    // Initialize needle properties
    void initialize_needle_properties(size_t needle_particles_num,
        size_t needle_elements_num, const Eigen::VectorXd&
        needle_particle_masses, double needle_bevel_tip_angle);

    // Update force interface
    void update(double real_time, const Eigen::MatrixXd& tissue_particles_pos,
        const Eigen::MatrixXd& needle_particles_pos, 
        const std::vector<Eigen::Quaterniond>& needle_elements_rot,
        const Eigen::MatrixXd& needle_particles_vel, 
        const std::vector<CollisionsInterface::NeedleCollisionInfo>&
        collision_info, const std::vector<int>& active_constraints_indices,
        utils::SystemState system_state);

    // Get tissue forces
    void get_external_tissue_forces(Eigen::MatrixXd& tissue_forces) {
        tissue_forces = m_tissue_forces;
    }

    // Get needle forces
    void get_external_needle_forces(Eigen::MatrixXd& needle_forces) {
        needle_forces = m_needle_forces;
    }

    // Get needle torques
    void get_external_needle_torques(Eigen::MatrixXd& needle_torques) {
        needle_torques = m_needle_torques;
    }


public:

    // Set insertion point indices
    void set_tissue_insertion_plane_point_indices(const Eigen::VectorXi&
        tissue_insertion_planes_indices){
            m_tissue_insertion_point_indices = tissue_insertion_planes_indices;
        }
    
private:
    
    // Physics parameters
    PhysicsParams* m_physics_params;

private:

    // Number of tissue particles
    size_t m_tissue_particles_num;

    // Tissue forces vector (N)
    Eigen::MatrixXd m_tissue_forces;

    // Tissue gravity force
    Eigen::MatrixXd m_tissue_grav_force;

    // Tissue masses (kg)
    Eigen::VectorXd m_tissue_masses;

    // Indices of tissue insertions plane points
    Eigen::VectorXi m_tissue_insertion_point_indices;

    // Current tissue tetrahedron position
    Eigen::MatrixXd m_tissue_tet_vertices;

    // Tissue tetrahedron mesh elements
    Eigen::MatrixXi m_tissue_tet_elements;

private:

    // Number of needle particles
    size_t m_needle_particles_num;

    // Number of needle elements
    size_t m_needle_elements_num;

    // Needle force vector (N)
    Eigen::MatrixXd m_needle_forces;

    // Needle torque vector (Nm)
    Eigen::MatrixXd m_needle_torques;

private:

    // Needle's bevel tip angle
    double m_nbta = 0.0;

    // Calculate cutting forces
    void calculate_cutting_forces(double real_time,
        const Eigen::MatrixXd& needle_particles_vel, 
        const std::vector<Eigen::Quaterniond>& needle_elements_rot,
        const std::vector<CollisionsInterface::NeedleCollisionInfo>&
        collision_info);

    // Cutting force first pass flag
    bool m_cutting_force_first_pass_flag = true;

    // Cutting force initial time
    double m_cutting_force_init_time = 0.0;

    // Cutting force magnituda
    double m_fc_mag = 0.9; // Class A (0.0105)
    // double m_fc_mag = 0.9; // Class B (0.021)
    // double m_fc_mag = 1.1; // Class C (0.032)
    // double m_fc_mag = 1.3; // Class D (0.048)
    // double m_fc_mag = 2.; // Class E  (0.054)
    // double m_fc_mag = 2.1; // Class F  (0.067)
    

    // Velocity sliding window
    std::shared_ptr<utils::SlidingWindow> m_vel_sliding_window;

private:

    // Friction model
    FrictionModel m_friction_model;

    // Calculate friction forces
    void calculate_friction_forces(double real_time,
        const Eigen::MatrixXd& needle_particles_vel, 
        const std::vector<Eigen::Quaterniond>& needle_elements_rot,
        const std::vector<CollisionsInterface::NeedleCollisionInfo>&
        collision_info, const std::vector<int>& active_constraints_indices);

    // Cutting force activation threshold 
    int m_cfa_thresh = 0;
};