#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "./mesh3D.h"
#include "gmsh.h"

class BoxStructuredMesh
{
public:
    BoxStructuredMesh(double width, double height, double depth,
        const Eigen::Vector3i& mesh_resolution, 
        const Eigen::Matrix3d& rot_mat=Eigen::Matrix3d::Identity(),
        const Eigen::Vector3d t_vec=Eigen::Vector3d::Zero());

    // Get boundary points
    Eigen::MatrixXd get_boundary_pts(void) { return m_pts; }

    // Get box mesh
    Mesh3D::TetMesh get_box_mesh(void) { return m_box_mesh; }

    // Get box surface mesh
    Mesh3D::SurfMesh get_box_surface(void) { return m_box_surf; }

private:
    
    // Volume width, height and depth 
    double m_width, m_height, m_depth; 

    // Volume points
    Eigen::MatrixXd m_pts;

    // Generate volume points
    static Eigen::MatrixXd generate_volume_points(double width,
        double height, double depth);

    // Generate structured mesh
    Mesh3D::TetMesh generate_structured_mesh(const Eigen::MatrixXd& 
        edge_pts, const Eigen::VectorXi& mesh_resolution);

    // Box mesh
    Mesh3D::TetMesh m_box_mesh;

private:

        // Triangle id and number of nodes per triangle (GMSH constants)
        const int m_triangle_id = 2;
        const int m_triangle_nodes = 3;

        // Tetrahedron id and number of nodes per tetrahedron (GMSH constants)
        const int m_tet_id = 4;
        const int m_tet_nodes = 4;

        // Extract elements
        Eigen::MatrixXi extract_elements(int element_type, int nodes_per_element,
            const std::vector<std::vector<std::size_t>>& element_node_tags);

private:

    // Surface mesh
    Mesh3D::SurfMesh m_box_surf;
};