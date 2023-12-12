#pragma once 

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <igl/edges.h>
#include <igl/barycenter.h>

#include "tissue_mesh.h"
#include "mesh3D.h"
#include "physics_params.hpp"
#include "force_interface.h"
#include "xpbd_constraints.h"

class TissueModel
{
public:
    TissueModel(const std::string& current_filepath,
        const std::string& block_filename, const Eigen::Vector3d& rot_F_F, 
        const Eigen::Vector3d& euler_ft, PhysicsParams* phyisics_params);

    // Update tissue model
    void predict_solution(double real_time, double dt,
        const Eigen::MatrixXd& fext);

    // Apply constraints 
    void constrain_iteration(double real_time, double dt);

    // Correct solution
    void update_solution(double real_time, double dt);

public:

    // Get tetrahedral mesh
    void get_tetrahedral_mesh(Mesh3D::TetMesh& tet_mesh) { tet_mesh = m_tet_mesh; }

    // Get number of particles
    int get_particles_number(void) { return m_particles_num; }

    // Get particles masses
    Eigen::VectorXd get_particles_masses(void) { return m_masses; }

    // Get particles inverse masses
    Eigen::VectorXd get_particles_inverse_masses(void) { return m_inv_masses; }

    // Insertion point indices
    Eigen::VectorXi get_insertion_points_indices(void) {
        return m_insertion_point_indices; }

    // Get boundary tissue points
    Eigen::MatrixXd get_tissue_boundary_points(void) {
        return m_tissue_boundary_pts;
    }
    

public:

    // Get tissue position prediction pointer
    Eigen::MatrixXd* get_position_prediction_ptr(void) {
        return &m_x_tilde;
    }

private:
    
    // Tissue mesh handle
    std::shared_ptr<TissueMesh> m_tissue_mesh; 

    // Current tetrahedral mesh
    Mesh3D::TetMesh m_tet_mesh;

    // Initial tetrahedral mesh
    Mesh3D::TetMesh m_initial_tet_mesh;

private:

    // Number of particles (n)
    int m_particles_num;

    // Number of tets
    int m_tets_num;

    // Current pos (n x 3)
    Eigen::MatrixXd m_pos;

    // Velocity (n x 3)
    Eigen::MatrixXd m_vel;
    
private:

    // Boundary point indices
    Eigen::VectorXi m_boundary_points_indices;

    // Indices of insertions plane points
    Eigen::VectorXi m_insertion_point_indices;

    // Tissue exterior normals
    Eigen::MatrixXd m_tissue_exterior_normals;

    // Boundary points
    Eigen::MatrixXd m_tissue_boundary_pts;
        
    // Bound compliance 
    double m_bc_compliance = 1.0e-10;

private:
        
    // Edges ids
    Eigen::MatrixXi m_edges_ids;

    // Rest volume
    Eigen::VectorXd m_rest_volume;

    // Edge lengths
    Eigen::VectorXd m_edge_lengths;

    // Masses 
    Eigen::VectorXd m_masses;

    // Inverted massses
    Eigen::VectorXd m_inv_masses;

    // Set point masses 
    void set_point_masses(void);

private:

    // Pointer to physics params
    PhysicsParams* m_physics_params_ptr;

private:

    // Volume constraints num
    int m_volume_constraints_num;

    // Edge constraints num
    int m_edge_contstraints_num;

    // Boundary constraints num
    int m_bc_constraints_num;    

private:
    // Tissue mesh dimensions
    Eigen::Vector3d m_tissue_mesh_dims;

private:

    // XPBD prediction
    Eigen::MatrixXd m_x_tilde;

    // Lagrange multipliers for edge contraints
    Eigen::VectorXd m_lambda_edge;

    // Lagrange multipliers for volume constraints
    Eigen::VectorXd m_lambda_volume;

    // Lagrange multipliers for boundary constraints
    Eigen::VectorXd m_lambda_bc;

    // Initialize lagrange multipliers
    void initialize_lagrange_multipliers(void);

    // Reset lagrange multipliers
    void reset_lagrange_multipliers(void);

    // Initialize xpbd FEM
    void initialize_xpbd_fem(void);

    // Lagrange multipliers for St Venant - Kirchhoff model
    std::vector<Eigen::Matrix<double, 6, 1>> m_lambda_svk;

    // Lagrange multipliers for Neo-Hookean hydrostatic model
    Eigen::VectorXd m_lambda_nhh;

    // Lagrange multipliers for Neo-Hookean deviatoric model
    Eigen::VectorXd m_lambda_nhd;

    // Inverse reference shape matrices (Dm^-1)
    std::vector<Eigen::Matrix3d> m_inv_dm_ref_shape_matrices;

    // Inverse matrices of elastic coefficients
    std::vector<Eigen::Matrix<double, 6, 6>> m_inv_elastic_coef_matrices;

    // Container of Lame' constatints 
    Eigen::VectorXd m_lame_mu, m_lame_lambda;

private:

    // Translational damping matrix
    Eigen::Matrix3d m_trans_damping_matrix;
};