#pragma once 

#include <iostream>
#include <memory>
#include <eigen3/Eigen/Dense>
#include "collisions_interface.h"

#include "physics_params.hpp"
#include "needle_model.h"
#include "tissue_model.h"
#include "mesh3D.h"
#include "xpbd_constraints.h"
#include "utils.h"

class NeedleTissueInterface
{
public:
    NeedleTissueInterface(const std::shared_ptr<TissueModel>& tissue_model, 
        const std::shared_ptr<NeedleModel>& needle_model, PhysicsParams*
        physics_params);

    // Update 
    void update(double real_time,
        const std::vector<CollisionsInterface::NeedleCollisionInfo>&
        collision_info, const std::vector<int>& active_constraints_indices,
        const std::vector<Eigen::Quaterniond>& needle_elements_rot,
        const Mesh3D::TetMesh& tet_mesh);

    // Constrain iteration
    void constrain_iteration(double real_time, double dt);

    // Get current system state
    utils::SystemState get_current_system_state(void) { return m_current_state; }

private:

    // Number of needle particles
    int m_np;

    // Number of needle quaternions
    int m_nq;

    // Initialize tissue model
    std::shared_ptr<TissueModel> m_tissue_model;

    // Needle model
    std::shared_ptr<NeedleModel> m_needle_model;

    // Physics parameters
    PhysicsParams* m_physics_params;

private:

    // Tissue particles inverse masses
    Eigen::VectorXd m_tissue_inv_masses;

    // Tissue elements
    Eigen::MatrixXi m_tissue_elements;

    // Needle particles inverse masses 
    Eigen::VectorXd m_needle_inv_masses;

    // Needle elements inertial tensors
    std::vector<Eigen::Matrix3d> m_needle_inertial_tensors;

private:

    /* Insertion state. Possible states are:
        1. State 0: Outside tissue
        2. State 1: Stiffness phase
        3. State 2: Insertion phase
        4. State 4: Retraction phase
    */ 
    utils::SystemState m_current_state = utils::SystemState::outside;

    // Update system state
    void update_system_state(void);

private:

    // Update stiffness phase
    void update_stiffness_phase(void);

    // First contact handling
    void update_first_contact_handler(const Mesh3D::TetMesh& tet_mesh);

    // Stiffness force threshold
    double m_stiffness_force_thresh = 0.3;
    
    // First contact tet id
    int m_first_contact_tet_id;

    // First contact natural coordinates
    Eigen::Vector4d m_first_contact_natural_coordinates;

    // First contact flag
    bool m_first_contact_flag = true;

private:

    // Check if we should solve contact (stiffness phase)
    bool m_stiffness_phase_on = false;

    // Needle-tissue interaction compliance
    double m_contact_compliance = 1.0e-10;

    // Contact normal
    Eigen::Vector3d m_contact_normal;

    // Lagrange multiplier for needle and tissue contact
    double m_lambda_tissue_contact, m_lambda_needle_contact;

    // Stiffness phase first pass flag
    bool m_stiffness_first_pass = true;

private:

    // Update insertion phase
    void update_insertion_phase(double real_time, const Mesh3D::TetMesh& tet_mesh,
        const std::vector<Eigen::Quaterniond>& needle_elements_rot);

    // Update lateral constraints
    void update_lateral_constraints(double real_time, const Mesh3D::TetMesh&
        tet_mesh, const std::vector<Eigen::Quaterniond>& needle_elements_rot);

    // Needle-tissue interaction compliance
    double m_nt_lateral_compliance = 1.0e-10;
    
    // Collision array
    Eigen::VectorXi m_collision_array;

    // Collision array rab_f_f_0
    Eigen::MatrixXd m_collision_array_rab_f_f_0;

    // Collision array for rotations rot_f0_F
    std::vector<Eigen::Matrix3d> m_collision_array_rot_f0_F;

    // L matrix
    Eigen::Matrix3d m_l_mat;

    // Active constraints indices
    std::vector<int> m_active_constraints_indices;

    // Collision info
    std::vector<CollisionsInterface::NeedleCollisionInfo> m_collision_info;

    // Lagrange multipliers for lateral needle/tissue interaction
    std::vector<Eigen::Vector3d> m_lambda_lateral_nti;
};
