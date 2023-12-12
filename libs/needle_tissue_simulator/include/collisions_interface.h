#pragma once 

#include <iostream>
#include <filesystem>
#include <memory>

#include <igl/barycenter.h>
#include "nanoflann.hpp"
#include "tetrahedron_collider.h"

class CollisionsInterface
{
public:
    // Constructor
    CollisionsInterface(const Eigen::MatrixXi& tissue_elements, 
        const Eigen::MatrixXd& needle_points);

    // Update
    void update(const Eigen::MatrixXd& tissue_vertices, 
        const Eigen::MatrixXd& tissue_barycenters, 
        const Eigen::MatrixXd& needle_points);

    // Get active needle points ids
    void get_active_needle_points_ids(std::vector<int>& ids) {
        ids = m_active_needle_points_ids;
    }

public:

    /**
     * @brief The struct provides information with regards to the collision 
     * status of a needle point. This includes the id of the needle point,
     * the inertial position of the point, its collision status and the 
     * tet element is in collision with (if there is a collision happening).
     * The possible collision states are the following:
     * 0: The point is outside the tetrahedron
     * 1: The point is inside the tetrahedron
     * 2: The point lies on the face f0 of the tetrahedron (f0: v0, v1, v3)
     * 3: The point lies on the face f1 of the tetrahedron (f1: v0, v2, v1)
     * 4: The point lies on the face f2 of the tetrahedron (f2: v3, v2, v0)
     * 5: The point lies on the face f3 of the tetrahedron (f3: v1, v2, v3)
     */ 

    struct NeedleCollisionInfo
    {
        // Needle point id
        int point_id;

        // Needle point position
        Eigen::Vector3d point_inertial_position;
    
        // Collision status
        int collision_status;

        // Tetrahedron element id
        int tet_element_id=-1;
    };

    // Get needle collision info
    void get_needle_collision_info(std::vector<NeedleCollisionInfo>& 
        needle_collision_info) {
        needle_collision_info = m_needle_collision_info;
    }

private:
    
    // Construct a kd-tree index
    // typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> m_kd_tree;
    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd,
        -1, nanoflann::metric_L2_Simple> m_kd_tree;

    /* Number of nearest neighbours. This variable is quite important 
    as it defines the quality of collision detection. It depends on the 
    number of tissue tetrahedrons. 30 seems to be working for the 
    current discretisation */
    int m_nn_num = 30;

private:

    // Tissue faces
    Eigen::MatrixXi m_tissue_elements;

    // Needle collision state
    std::vector<NeedleCollisionInfo> m_needle_collision_info;

    // Active needle points ids
    std::vector<int> m_active_needle_points_ids;
};
