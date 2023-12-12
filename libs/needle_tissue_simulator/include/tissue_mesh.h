#pragma once 

#include <iostream>
#include <memory>
#include <filesystem>

#include <igl/read_triangle_mesh.h>
#include <igl/writeOBJ.h>
#include <igl/readOBJ.h>
#include <igl/per_vertex_normals.h>
#include <eigen3/Eigen/Dense>
#include <dynamics_math_eigen/dynamics_math_eigen.h>

#include "./mesh3D.h"
#include "./box_structured_mesh.h"

class TissueMesh
{
public:
    TissueMesh(const std::string& current_filepath,
        const Eigen::Vector3d& rot_F_F, 
        const Eigen::Vector3d& euler_ft, 
        const Eigen::Vector3d& block_dims);

    // Get model mesh
    std::shared_ptr<Mesh3D> get_model_mesh(void) { return m_model_mesh; }

public:

    // Get indices of boundary points
    Eigen::VectorXi get_boundary_points_indices(void) {
        return m_boundary_indices;
    }

    // Get insertion plane point indices
    Eigen::VectorXi get_insertion_plane_point_indices(void){
        return m_insertion_plane_indices;
    }

    // Get boundary normals
    Eigen::MatrixXd get_tissue_exterior_normals(void) {
        return m_tissue_exterior_normals; 
    }

    // Get boundary points
    Eigen::MatrixXd get_tissue_boundary_points(void) {
        return m_tissue_boundary_points; 
    }

private:

    // Prostate mesh
    std::shared_ptr<Mesh3D> m_prostate_mesh; 

    // Prostate mesh decimation
    double m_prostate_decimation = 0.1;

private:

    // Block mesh width, height and depth (m)
    double m_block_width = 0.2;
    double m_block_height = 0.15;
    double m_block_depth = 0.24;
        
    // Generate box structured mesh
    std::shared_ptr<BoxStructuredMesh> m_box_structured_mesh;

    // Block mesh
    std::shared_ptr<Mesh3D> m_block_mesh;

    // Block resolution (width, height, depth)
    Eigen::Vector3i m_block_res = Eigen::Vector3i(8, 8, 8);

private:

    // Model mesh
    std::shared_ptr<Mesh3D> m_model_mesh;

    // Model surface mesh
    Mesh3D::SurfMesh m_model_surf;

    // Model volume mesh
    Mesh3D::TetMesh m_model_vol;

private:

    // Generate tissue boundary conditions
    void generate_tissue_boundary_conditions(const Eigen::MatrixXd& model_vertices);

    // Mesh boundary indices
    Eigen::VectorXi m_boundary_indices;

    // Insertion plane indices
    Eigen::VectorXi m_insertion_plane_indices;

    // Tissue exterior normals (wrt to the tissue insertion frame fd)
    Eigen::MatrixXd m_tissue_exterior_normals;

    // Tissue boundary points
    Eigen::MatrixXd m_tissue_boundary_points;

private:

    // Tissue rotation matrix with respect to the inertial frame
    Eigen::MatrixX3d m_rot_ft_F;

private:
    
    // Transform mesh
    Eigen::MatrixXd transform_mesh(const Eigen::MatrixXd& vert, 
        const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos);
};
