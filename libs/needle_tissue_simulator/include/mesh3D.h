#pragma once 

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <igl/read_triangle_mesh.h>
#include "./tetrahedral_meshing.h"

class Mesh3D
{
public:
    // Tet mesh
    struct TetMesh
    {
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi elements;
    };

    // Surface mesh
    struct SurfMesh
    {
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
    };
    
    // Constructor
    Mesh3D(const SurfMesh& visual_mesh, bool decimation,
        double decimation_coefficient=1.0,
        const Eigen::Matrix3d& rot=Eigen::Matrix3d::Identity(),
        const Eigen::Vector3d& pos=Eigen::Vector3d::Zero());

    // Constructor
    Mesh3D(const TetMesh& tet_mesh, const SurfMesh& surf_mesh,
        const Eigen::Matrix3d& rot=Eigen::Matrix3d::Identity(),
        const Eigen::Vector3d& pos=Eigen::Vector3d::Zero());
    
public:
    // Get tetrahedral mesh
    TetMesh get_tetrahedral_mesh(void) {return m_tet_mesh; }
    
    // Get surface mesh
    SurfMesh get_surface_mesh(void) { return m_surf_mesh; }
    
    // Get visual mesh
    SurfMesh get_visual_mesh(void) { return m_visual_mesh; }

    // Get flattened tetrahedral mesh
    static SurfMesh get_flattened_tetrahedral_mesh(const Mesh3D::TetMesh& tet_mesh);

    // Get flattened tetrahedral mesh
    SurfMesh get_flattened_tetrahedral_mesh(void);

    // Transform mesh
    static Eigen::MatrixXd transform_mesh(const Eigen::MatrixXd& vert, 
        const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos);

private:
    
    // Current tetrahedral mesh
    TetMesh m_tet_mesh;
    
    // Current surface mesh
    SurfMesh m_surf_mesh;
    
    // Initial tetrahedral mesh
    TetMesh m_initial_tet_mesh;

    // Visual mesh
    SurfMesh m_visual_mesh;

private:

    // Tetrahedral mesh handle
    std::shared_ptr<TetrahedralMeshing> m_tet_meshing_ptr;
};
