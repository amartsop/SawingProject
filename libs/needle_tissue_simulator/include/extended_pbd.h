#pragma once 

#include <iostream>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <armadillo>
#include <igl/edges.h>

#include "./mesh3D.h"
#include "./physics_params.hpp"
#include "./force_interface.h"

namespace st
{
    class ExtendedPBD
    {
    public:
        ExtendedPBD(const st::Mesh3D::TetMesh& tet_mesh,
            PhysicsParams* physics_params,
            const Eigen::VectorXi& boundary_point_indices,
            const Eigen::VectorXi& insertion_point_indices,
            const Eigen::MatrixXd& exterior_normals);
    
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

        // struct NeedleCollisionInfo
        // {
        //     // Needle point id
        //     int point_id;

        //     // Needle point position
        //     Eigen::Vector3d point_inertial_position;
        
        //     // Collision status
        //     int collision_status;

        //     // Tetrahedron element id
        //     int tet_element_id;
        // };

        // Update mesh
        void update(double t);

        // Get tetrahedral mesh
        st::Mesh3D::TetMesh get_tetrahedral_mesh(void) {return m_tet_mesh; }
    
        // Get initial mesh tetrahedral mesh
        st::Mesh3D::TetMesh get_initial_tetrahedral_mesh(void) { return m_initial_tet_mesh; }

        // // Set force info
        // void set_force_info(const std::vector<st::ForceInterface::ForceInfo>&
        //     force_info)
        // {
        //     m_force_ptr->set_force_info(force_info);
        // }

    
    private:

        // Boundary point indices
        Eigen::VectorXi m_boundary_points_indices;

        // Indices of insertions plane points
        Eigen::VectorXi m_insertion_point_indices;

        // Tissue exterior normals
        Eigen::MatrixXd m_tissue_exterior_normals;

    private:

        // Mesh initial position
        Eigen::Vector3d m_mesh_init_pos = {0.0, 0., 0.0};
    
        // Current tetrahedral mesh
        st::Mesh3D::TetMesh m_tet_mesh;
    
        // Initial tetrahedral mesh
        st::Mesh3D::TetMesh m_initial_tet_mesh;

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

        // Edge compliance
        double m_edge_compliance = 1.e-4;

        // Volume compliance
        double m_vol_compliance = 0.0;
        
        // Bound compliance 
        double m_bc_compliance = 1.0e-10;
        
        // Needle-tissue interaction compliance
        double m_nt_compliance = 1.0e-10;
        
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

    // private:
    //     // Set point masses 
    //     void set_point_masses(void);

    //     // Calculate tetrahedron volume
    //     double tetrahedron_volume(const Eigen::Vector3d& p0,
    //         const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
    //         const Eigen::Vector3d& p3);

    // private:
    //     // Pointer to physics params
    //     PhysicsParams* m_physics_params_ptr;

    //     // Pointer to force interface
    //     std::shared_ptr<st::ForceInterface> m_force_ptr;

    //     // Position prediction (presolve)
    //     void position_prediction(double dts, double real_time);

    // private:

    //     // Solve edges
    //     void solve_edge_constraints(double dts, Eigen::MatrixXd& x_tilde, 
    //         Eigen::VectorXd& lambda_edge);

    //     // Solve volume constraints
    //     void solve_volume_constraints(double dts, Eigen::MatrixXd& x_tilde,
    //         Eigen::VectorXd& lambda_volume);
    
    //     // Solve boundary constraints
    //     void solve_boundary_constraints(double real_time, double dts, 
    //         Eigen::MatrixXd& x_tilde, Eigen::VectorXd& lambda_bc);
    
    //     // Solve needle tissue interaction constraints
    //     void solve_needle_tissue_interation_constraints(const
    //         std::vector<NeedleCollisionInfo>& collision_info,
    //         const std::vector<int>& active_constraints_indices,
    //         double dts, Eigen::MatrixXd& x_tilde, Eigen::VectorXd& lambda_nt);

    
    // private:
    //     // Simulation timestep
    //     double m_dt = 0.001;

    //     // Iterations number 
    //     int m_iterations_num = 1e2;
    
    // private:
    
    //     // Volume constraints num
    //     int m_volume_constraints_num;
    
    //     // Edge constraints num
    //     int m_edge_contstraints_num;
    
    //     // Boundary constraints num
    //     int m_bc_constraints_num;    

    //     // Tolerance
    //     double m_eps = 1.0e-6;
    
    // private:

    //     // Update collision arrays        
    //     void update_nt_collision_array(const std::vector<NeedleCollisionInfo>&
    //         collision_info, const std::vector<int>& active_constraints_indices);

    //     // Collision array
    //     Eigen::VectorXi m_collision_array;
    
    //     // Collision array rotations
    //     std::vector<Eigen::Matrix3d> m_collision_array_rotation;
    
    //     // Collision array rab_f_f_0
    //     Eigen::MatrixXd m_collision_array_rab_f_f_0;
    
    //     // L matrix
    //     Eigen::Matrix3d m_l_mat;
    };
}