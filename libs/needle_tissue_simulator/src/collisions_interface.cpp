#include "../include/collisions_interface.h"

// Initialize 
CollisionsInterface::CollisionsInterface(const Eigen::MatrixXi& tissue_elements, 
    const Eigen::MatrixXd& needle_points)
{
    // Tissue elements
    m_tissue_elements = tissue_elements;

    // Initialize needle collision info
    m_needle_collision_info.resize(needle_points.rows());
}

// Update
void CollisionsInterface::update(const Eigen::MatrixXd& tissue_vertices, 
    const Eigen::MatrixXd& tissue_barycenters,
    const Eigen::MatrixXd& needle_points)
{
    // Construct mat index
    m_kd_tree mat_index(3, std::cref(tissue_barycenters), 10 /* max leaf */);

    // Initialize active needle points
    m_active_needle_points_ids.resize(0);
    
    // Loop through all needle points and find nearest neighbours
    for (size_t i = 0; i < needle_points.rows(); i++)
    {
        // Get needle point position
        Eigen::Vector3d point_position = needle_points.row(i);

        // Set needle point id
        m_needle_collision_info.at(i).point_id = i;

        // Set needle point postion
        m_needle_collision_info.at(i).point_inertial_position = point_position;

        // Initialize collision status to no collision
        m_needle_collision_info.at(i).collision_status = 0;
        
        // Define query point
        std::vector<double> query_pt = {point_position(0), point_position(1), 
            point_position(2)};

        // Do a knn search
        std::vector<size_t> pts_indices(m_nn_num);
        std::vector<double> out_dists_sqr(m_nn_num);
        nanoflann::KNNResultSet<double> resultSet(m_nn_num);

        resultSet.init(&pts_indices[0], &out_dists_sqr[0]);
        mat_index.index->findNeighbors(
            resultSet, &query_pt[0], nanoflann::SearchParams(10));   

        // Loop the nearest neighbours and search for any collisions
        for (size_t j = 0; j < pts_indices.size(); j++)
        {
            // Get tetrahedron id
            size_t tet_id = pts_indices.at(j);

            // Get tet vertices indices
            Eigen::VectorXi tet_vertex_indices = m_tissue_elements.row(tet_id);
        
            // Generate tetrahedron vertices
            Eigen::MatrixXd tet_vertices = Eigen::MatrixXd::Zero(4, 3);
            for (size_t k = 0; k < tet_vertex_indices.rows(); k++)
            {
                // Define tet vertices
                tet_vertices.row(k) = tissue_vertices.row(tet_vertex_indices(k));
            }

            // Check for collision 
            int collision_state =
                TetrahedronCollider::check_point_collision(tet_vertices, point_position);
        
            /* If there is a collision, with any of the nearest neighbours,
            then assign it to collision info */
            if (collision_state)
            {
                m_active_needle_points_ids.push_back(i);
                m_needle_collision_info.at(i).collision_status = collision_state;
                m_needle_collision_info.at(i).tet_element_id = tet_id;
            }
        }
    }
}