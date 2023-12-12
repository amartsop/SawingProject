#include "../include/tissue_mesh.h"

TissueMesh::TissueMesh(const std::string& current_filepath,
    const Eigen::Vector3d& rot_F_F, const Eigen::Vector3d& euler_ft,
    const Eigen::Vector3d& block_dims)
{
    // Set tissue block dimensions
    m_block_depth = block_dims(0);
    m_block_width = block_dims(1);
    m_block_height = block_dims(2);
    
    /******************* Tissue block ***************************/
    // Tissue block transform    
    Eigen::Matrix3d block_rot_mat =
        dme::EulerRotations::basic_rotation_x(M_PI/2.0) * 
        dme::EulerRotations::basic_rotation_y(M_PI/2.0);
    
    // Generate tissue mesh    
    BoxStructuredMesh box_mesh = BoxStructuredMesh(m_block_width,
        m_block_height, m_block_depth, m_block_res, block_rot_mat);

    // Tissue rotation matrix with respect to the inertial frame
    Eigen::Matrix3d rot_ft_F = dme::EulerRotations::rotation(euler_ft);

    // Generate 3D block mesh
    m_model_mesh = std::make_shared<Mesh3D>(box_mesh.get_box_mesh(),
        box_mesh.get_box_surface(), rot_ft_F, rot_F_F);

    // Define model surface mesh
    m_model_surf = m_model_mesh->get_flattened_tetrahedral_mesh();

    // Generate tissue boundaries (before transforming)
    generate_tissue_boundary_conditions(box_mesh.get_box_mesh().vertices);
}

// Generate tissue boundary conditions
void TissueMesh::generate_tissue_boundary_conditions(const Eigen::MatrixXd&
    model_vertices)
{
    /* Here we assume that only the front and top faces of the tissue are
    free to move and the rest faces are constrained. */

    // Free indices
    std::vector<int> free_indices;

    // Boundary indices
    std::vector<int> boundary_indices;

    // Insertion plane indices
    std::vector<int> insertion_plane_indices;

    // Initialize exterior normals
    m_tissue_exterior_normals = Eigen::MatrixXd::Zero(model_vertices.rows(), 3);

    // Loop through all vertices and find the exterior points and their normals
    for (size_t i = 0; i < m_tissue_exterior_normals.rows(); i++)
    {
        // Get vertex i
        Eigen::Vector3d vertex_i = model_vertices.row(i).transpose();
        
        // Insertion plane flag
        bool insertion_plane = (vertex_i(0) == - m_block_depth / 2.0) &&
            (abs(vertex_i(1)) < m_block_width / 2.0) &&
            (abs(vertex_i(2)) < m_block_height / 2.0);
    
        // Normals in the x direction
        if (insertion_plane) 
        {
            // This is the insertion plane
            m_tissue_exterior_normals(i, 0) = - 1;
        
            // Add index to the insertion plane indices
            insertion_plane_indices.push_back(i);

            continue;
        }
        if (vertex_i(0) == m_block_depth / 2.0) 
        {
            m_tissue_exterior_normals(i, 0) = 1;
            boundary_indices.push_back(i);
        }
        // Normals in the y direction
        if (vertex_i(1) == m_block_width / 2.0) 
        {
            m_tissue_exterior_normals(i, 1) = 1;
            boundary_indices.push_back(i);
        }
        if (vertex_i(1) == - m_block_width / 2.0) 
        {
            m_tissue_exterior_normals(i, 1) = -1;
            boundary_indices.push_back(i);
        }
        // Normals in the z direction
        if (vertex_i(2) == m_block_height / 2.0) 
        {
            m_tissue_exterior_normals(i, 2) = 1;
            // boundary_indices.push_back(i);
        }
        if (vertex_i(2) == - m_block_height / 2.0) 
        {
            m_tissue_exterior_normals(i, 2) = -1;
            boundary_indices.push_back(i);
        }
    }

    // Normalize(unit norm) normal vectors and transform them
    for (size_t i = 0; i < m_tissue_exterior_normals.rows(); i++)
    {
        // Get vertex i
        Eigen::RowVector3d vertex_i = m_tissue_exterior_normals.row(i);

        if (vertex_i.norm() > 1)
        {
            m_tissue_exterior_normals.row(i) = vertex_i.normalized();
        }
    }

    // Get only the unique boundary indices
    auto it = std::unique(boundary_indices.begin(), boundary_indices.end());
    boundary_indices.resize(distance(boundary_indices.begin(), it));

    // Convert boundary indices to eigen vector
    m_boundary_indices = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(
        boundary_indices.data(), boundary_indices.size());

    // Convert insertion plane indices to eigen vector
    m_insertion_plane_indices = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(
        free_indices.data(), free_indices.size());

    // Generate the tissue boundary points
    m_tissue_boundary_points = Eigen::MatrixXd::Zero(m_boundary_indices.rows(), 3);

    auto tissue_mesh = m_model_mesh->get_tetrahedral_mesh();

    for (size_t i = 0; i < m_boundary_indices.rows(); i++)
    {
        m_tissue_boundary_points.row(i) =
            tissue_mesh.vertices.row(m_boundary_indices(i));
    }
}