#include "../include/mesh3D.h"

// Constructor
Mesh3D::Mesh3D(const SurfMesh& visual_mesh, bool decimation,
    double decimation_coefficient, const Eigen::Matrix3d& rot,
    const Eigen::Vector3d& pos)
{
    // Set visual mesh
    m_visual_mesh = visual_mesh;

    // Generate tetrahedral mesh
    m_tet_meshing_ptr = std::make_shared<TetrahedralMeshing>(
        m_visual_mesh.vertices, m_visual_mesh.faces, decimation,
        decimation_coefficient);
    
    // Set tet mesh
    m_tet_mesh.vertices = transform_mesh(
        m_tet_meshing_ptr->get_tetrahedral_vertices(), rot, pos);

    m_tet_mesh.elements = m_tet_meshing_ptr->get_tetrahedral_elements();

    // Set surface mesh
    m_surf_mesh.vertices = transform_mesh(
        m_tet_meshing_ptr->get_surface_vertices(), rot, pos);

    m_surf_mesh.faces = m_tet_meshing_ptr->get_surface_faces();
}

// Constructor
Mesh3D::Mesh3D(const Mesh3D::TetMesh& tet_mesh,
    const Mesh3D::SurfMesh& surf_mesh, const Eigen::Matrix3d& rot,
    const Eigen::Vector3d& pos)
{
    // Set tet mesh
    m_tet_mesh.vertices = transform_mesh(tet_mesh.vertices, rot, pos);
    m_tet_mesh.elements = tet_mesh.elements; 

    // Set surface mesh
    m_surf_mesh.vertices = transform_mesh(surf_mesh.vertices, rot, pos);
    m_surf_mesh.faces = surf_mesh.faces;
}


// Converts a tetrahedral mesh to a flattend version for plotting.
// Each tetrahedral is treated as four triangles.
Mesh3D::SurfMesh Mesh3D::get_flattened_tetrahedral_mesh(const
    Mesh3D::TetMesh& tet_mesh)
{
    // Initialize flat mesh
    Mesh3D::SurfMesh flat_mesh;

    // Elements size
    size_t elements_size = tet_mesh.elements.rows();

    // Initialize flat matrices
    flat_mesh.vertices = Eigen::MatrixXd::Zero(elements_size*4, 3);
    flat_mesh.faces = Eigen::MatrixXi::Zero(elements_size*4, 3);
    
    // Generate them
    for (size_t i = 0; i < tet_mesh.elements.rows(); i++)
    {
        flat_mesh.vertices.row(i*4+0) = tet_mesh.vertices.row(tet_mesh.elements(i,0));
        flat_mesh.vertices.row(i*4+1) = tet_mesh.vertices.row(tet_mesh.elements(i,1));
        flat_mesh.vertices.row(i*4+2) = tet_mesh.vertices.row(tet_mesh.elements(i,2));
        flat_mesh.vertices.row(i*4+3) = tet_mesh.vertices.row(tet_mesh.elements(i,3));
        flat_mesh.faces.row(i*4+0) << (i*4)+0, (i*4)+1, (i*4)+3;
        flat_mesh.faces.row(i*4+1) << (i*4)+0, (i*4)+2, (i*4)+1;
        flat_mesh.faces.row(i*4+2) << (i*4)+3, (i*4)+2, (i*4)+0;
        flat_mesh.faces.row(i*4+3) << (i*4)+1, (i*4)+2, (i*4)+3;
    }

    return flat_mesh;
}

// Converts a tetrahedral mesh to a flattend version for plotting.
// Each tetrahedral is treated as four triangles.
Mesh3D::SurfMesh Mesh3D::get_flattened_tetrahedral_mesh(void)
{
    // Initialize flat mesh
    Mesh3D::SurfMesh flat_mesh;

    // Elements size
    size_t elements_size = m_tet_mesh.elements.rows();

    // Initialize flat matrices
    flat_mesh.vertices = Eigen::MatrixXd::Zero(elements_size*4, 3);
    flat_mesh.faces = Eigen::MatrixXi::Zero(elements_size*4, 3);
    
    // Generate them
    for (size_t i = 0; i < m_tet_mesh.elements.rows(); i++)
    {
        flat_mesh.vertices.row(i*4+0) = m_tet_mesh.vertices.row(m_tet_mesh.elements(i,0));
        flat_mesh.vertices.row(i*4+1) = m_tet_mesh.vertices.row(m_tet_mesh.elements(i,1));
        flat_mesh.vertices.row(i*4+2) = m_tet_mesh.vertices.row(m_tet_mesh.elements(i,2));
        flat_mesh.vertices.row(i*4+3) = m_tet_mesh.vertices.row(m_tet_mesh.elements(i,3));
        flat_mesh.faces.row(i*4+0) << (i*4)+0, (i*4)+1, (i*4)+3;
        flat_mesh.faces.row(i*4+1) << (i*4)+0, (i*4)+2, (i*4)+1;
        flat_mesh.faces.row(i*4+2) << (i*4)+3, (i*4)+2, (i*4)+0;
        flat_mesh.faces.row(i*4+3) << (i*4)+1, (i*4)+2, (i*4)+3;
    }

    return flat_mesh;
}

// Tranform mesh
Eigen::MatrixXd Mesh3D::transform_mesh(const Eigen::MatrixXd& vert, 
    const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos)
{
    // Initialize new vertices
    Eigen::MatrixXd vert_new = Eigen::MatrixXd::Zero(vert.rows(), 3);

    for (size_t i = 0; i < vert_new.rows(); i++)
    {
        vert_new.row(i) = (pos + rot * vert.row(i).transpose()).transpose();
    }

    return vert_new;
}