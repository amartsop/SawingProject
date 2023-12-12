#pragma once 

#include <iostream>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <igl/barycenter.h>

#include "./include/mesh3D.h"
#include "./include/physics_params.hpp"
#include "./include/collisions_interface.h"
#include "./include/tissue_model.h"
#include "./include/needle_model.h"
#include "./include/needle_tissue_interface.h"
#include "utils.h"

class NeedleTissueSimulator
{
public:
    NeedleTissueSimulator(double ts, const std::string& current_filepath,
        const std::string& block_filename, const Eigen::Vector3d& roa_F_F,
        const Eigen::Vector3d& euler_f, const Eigen::Vector3d& rot_F_F,
        const Eigen::Vector3d& euler_ft);

    // Update 
    void update(double real_time,
        const Eigen::Vector3d& roa_F_F=Eigen::Vector3d::Zero(),
        const Eigen::Vector3d& eul_base=Eigen::Vector3d::Zero());

private:

    // Initialize tissue model
    std::shared_ptr<TissueModel> m_tissue_model;

    // Needle model
    std::shared_ptr<NeedleModel> m_needle_model;

    // Force interface
    std::shared_ptr<ForceInterface> m_force_interface;

private:

    // Tissue mesh
    Mesh3D::TetMesh m_tissue_tet_mesh;

    // Needle state
    Eigen::MatrixXd m_needle_particles_pos;
    std::vector<Eigen::Quaterniond> m_needle_elements_rotation;

    // Needle velocities
    Eigen::MatrixXd m_needle_particles_vel, m_needle_elements_rot_vel;

public:

    // Get tissue vertices
    void get_tissue_vertices(Eigen::MatrixXd& tissue_vertices);

    // Get tissue faces
    void get_tissue_faces(Eigen::MatrixXi& tissue_faces);

    // Get needle state
    void get_needle_state(Eigen::MatrixXd& needle_particles_position,
        std::vector<Eigen::Quaterniond>& needle_elements_rotations);

    // Get needle base state
    void get_needle_base_state(Eigen::Vector3d& needle_pos,
        Eigen::Vector3d& needle_eul);

    // Get boundary tissue points 
    void get_tissue_boundary_points(Eigen::MatrixXd& boundary_pts);
    
    // Get needle visual vertices
    void get_needle_visual_vertices(const Eigen::Vector3d& rap_f_f_0,  
        Eigen::Vector3d& rop_F_F);

    // Get needle reaction force
    void get_needle_generalized_reaction_force(Eigen::Vector3d& force, 
        Eigen::Vector3d& moment);

public:

    // Get in tissue needle points
    void get_in_tissue_needle_points(std::vector<int>&
        in_tissue_needle_pts_indices)
    {
        in_tissue_needle_pts_indices = m_active_needle_points_indices;
    }
        
private:

    // Physics params
    PhysicsParams m_physics_params;

    // Simulation timestep
    double m_dt;

    // Number of iterations (should always be one in case of substeping)
    int m_iterations_num = 1;

    // Substeps num
    int m_substeps_num = 100.;

private:

    // Collisions interface
    std::shared_ptr<CollisionsInterface> m_collisions_interface;

    // Needle tissue interface model
    std::shared_ptr<NeedleTissueInterface> m_needle_tissue_interface;

    // Needle collision info
    std::vector<CollisionsInterface::NeedleCollisionInfo> m_needle_collision_info;

    // Active constraitns indices
    std::vector<int> m_active_needle_points_indices;


};