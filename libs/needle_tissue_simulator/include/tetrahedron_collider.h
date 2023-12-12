#pragma once 
#include <iostream>

#include <eigen3/Eigen/Dense>

class TetrahedronCollider
{
public:
    TetrahedronCollider(/* args */) {};

    /**
     * @brief Check for collision between a point a tetrahedron
     * 
     * @param tet_vertices Is a 4x3 matrix containing the tetrahedron
     * vertices in a row format.
     * @param point_pos The point we are checking against.
     * @return int The integer represents the state of collision. The possible 
     * states are the following
     * 0: The point is outside the tetrahedron
     * 1: The point is inside the tetrahedron
     */ 
    static int check_point_collision(const Eigen::MatrixXd& tet_vertices, 
        const Eigen::Vector3d& point_pos, double tol=double(0));

private:
        
    /**
     * @brief Defines the 4 faces of the tetrahedron
     * 
     * @param tet_vertices The 4 vertices of the tetrahedron
     * @return std::vector<Eigen::Matrix3d> Tetrahedron faces. Each element of
     * the vector contains a matrix which in turn contains the 3 vertices
     * of the triangular face (4 in total). Each matrix row is a vertex
     */ 
    static std::vector<Eigen::Matrix3d> define_tetrahedron_faces(const 
        Eigen::MatrixXd& tet_vertices);

    /**
     * @brief Calculates the normals for each of the tetrahedrons faces
     * 
     * @param tet_faces The tetrahedrons faces as defined by @define_tetrahedron_faces
     * @return std::vector<Eigen::Vector3d> Each element contains a face normal
     * (n0, n1, n2, n3)
     */
    static std::vector<Eigen::Vector3d> calculate_face_normals(const 
        std::vector<Eigen::Matrix3d>& tet_faces);

    /**
     * @brief Calculates the coefficients that define the plane equation for
     * each face of the tetrahedron. The equation is of the form ax + by + cz + d
     * @param tetrahedron_faces The faces of tetrahedron as defined in 
     * @define_tetrahedron_faces
     * @param face_normals Each element contains the normal vector of the face i
     * @param tet_barycenter The barycenter of the tetrahedron. Is used for 
     * checking the direction of the plane. Each plane's normal should 
     *  be pointing towards the exterior of the tetrahedron.
     * @return std::vector<Eigen::Vector4d> Each element contains the
     * coefficients of the plane
     */
    static std::vector<Eigen::Vector4d> calculate_plane_coefficients(
        std::vector<Eigen::Matrix3d>& tetrahedron_faces,
        const std::vector<Eigen::Vector3d>& face_normals, const
        Eigen::Vector3d& tet_barycenter);

    // Plane equation function
    static double plane_equation(const Eigen::Vector3d& point_pos,
        const Eigen::Vector4d& plane_equations_coeffs);

    // Simple sign function
    inline static int sgn(double val, double tol=double(0)) {
        int sgn_val = 0;
        if (val > tol) { sgn_val = 1; }
        if (val < tol) { sgn_val = -1; }
        if (std::abs(val) < tol) { sgn_val = 0; }
        return sgn_val;
    }
};