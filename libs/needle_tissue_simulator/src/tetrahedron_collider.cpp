#include "../include/tetrahedron_collider.h"

// Check for collision between a point a tetrahedron
int TetrahedronCollider::check_point_collision(const Eigen::MatrixXd&
    tet_vertices, const Eigen::Vector3d& point_pos, double tol)
{
    // Calculate barycenter
    Eigen::Vector3d barycenter = 0.25 * Eigen::Vector3d(tet_vertices.col(0).sum(),
        tet_vertices.col(1).sum(), tet_vertices.col(2).sum());

    // Calculate tetrahedron faces
    std::vector<Eigen::Matrix3d> tetrahedron_faces = define_tetrahedron_faces(
        tet_vertices);

    // Calculate face normals
    std::vector<Eigen::Vector3d> face_normals = calculate_face_normals(
        tetrahedron_faces);

    // Calculate plane coefficients
    std::vector<Eigen::Vector4d> face_plane_coefficients =
        calculate_plane_coefficients(tetrahedron_faces, face_normals, barycenter);

    // Evaluate plane equations on the point
    Eigen::Vector4d plane_evaluations = Eigen::Vector4d::Zero();
    
    for (size_t i = 0; i < face_plane_coefficients.size(); i++)
    {
        plane_evaluations(i) = plane_equation(point_pos,
            face_plane_coefficients.at(i));
    }

    // Calculate collision state     
    int collision_state = 0;

    // Point outside tetrahedron flag
    bool outside_flag = (!std::signbit(plane_evaluations(0))) &&
        (!std::signbit(plane_evaluations(1))) &&
        (!std::signbit(plane_evaluations(2))) && 
        (!std::signbit(plane_evaluations(3)));

    if (outside_flag) {
        collision_state = 0;
    }

    // Point inside tetrahedron flag
    bool inside_flag = (std::signbit(plane_evaluations(0))) &&
        (std::signbit(plane_evaluations(1))) &&
        (std::signbit(plane_evaluations(2))) && 
        (std::signbit(plane_evaluations(3)));
    
    if (inside_flag)
    {
        collision_state = 1;
    }
    
    return collision_state;
}

// Calculate the coefficients of the plane equation that defines each face 
// of the tetrahedron
std::vector<Eigen::Vector4d> TetrahedronCollider::calculate_plane_coefficients(
    std::vector<Eigen::Matrix3d>& tetrahedron_faces,
    const std::vector<Eigen::Vector3d>& face_normals, const Eigen::Vector3d&
    tet_barycenter)
{
    // Intitialize plane coefficiencts
    std::vector<Eigen::Vector4d> face_plane_coefficients(face_normals.size());

    // Loop through faces and calculate coefficients
    for (size_t i = 0; i < face_normals.size(); i++)
    {
        // Find triangle plane equation coefficients
        double a = face_normals.at(i)(0);
        double b = face_normals.at(i)(1);
        double c = face_normals.at(i)(2);
        double d = - (a * tetrahedron_faces.at(i)(0, 0) +
            b * tetrahedron_faces.at(i)(0, 1) + c * tetrahedron_faces.at(i)(0, 2));

        // Define plane coeffiencts
        Eigen::Vector4d plane_coeffs = Eigen::Vector4d{a, b, c, d};

        /* Check if the coeffients are such so that the normal of each face points 
        to the exterior of the tetrahedron. To do this we check the plane 
        equation against the tetrahedrons barycenter (which always lies 
        on the interior of the tetrahedron) */
        double equation_evaluation = plane_equation(tet_barycenter, plane_coeffs);
    
        // Adjust coefficients depending on the barycenters evaluation
        if (sgn(equation_evaluation) <= 0)
        {
            face_plane_coefficients.at(i) = plane_coeffs;
        }
        else
        {
            face_plane_coefficients.at(i) = - plane_coeffs;
        }
    }

    return face_plane_coefficients;
}

// Calculate the normals for each of the tetrahedron's faces
std::vector<Eigen::Vector3d> TetrahedronCollider::calculate_face_normals(const 
    std::vector<Eigen::Matrix3d>& tet_faces)
{
    // Initialize face normals
    std::vector<Eigen::Vector3d> face_normals(tet_faces.size());

    // Loop through faces and find their normals
    for (size_t i = 0; i < tet_faces.size(); i++)
    {
        // First triangle vector 
        Eigen::Vector3d v = tet_faces.at(i).row(1) - tet_faces.at(i).row(0);

        // Second triangle vector 
        Eigen::Vector3d w = tet_faces.at(i).row(2) - tet_faces.at(i).row(0);

        // Normals
        face_normals.at(i) = v.cross(w).normalized();
    }
    
    return face_normals;
}

// Defines the 4 faces of the tetrahedron
std::vector<Eigen::Matrix3d> TetrahedronCollider::define_tetrahedron_faces(
    const Eigen::MatrixXd& tet_vertices)
{
    // Initialize tetrahedron faces
    std::vector<Eigen::Matrix3d> tet_faces(4);

    // Face 0
    tet_faces.at(0).row(0) = tet_vertices.row(0);
    tet_faces.at(0).row(1) = tet_vertices.row(1);
    tet_faces.at(0).row(2) = tet_vertices.row(3);
    
    // Face 1
    tet_faces.at(1).row(0) = tet_vertices.row(0);
    tet_faces.at(1).row(1) = tet_vertices.row(2);
    tet_faces.at(1).row(2) = tet_vertices.row(1);

    // Face 2
    tet_faces.at(2).row(0) = tet_vertices.row(3);
    tet_faces.at(2).row(1) = tet_vertices.row(2);
    tet_faces.at(2).row(2) = tet_vertices.row(0);

    // Face 3
    tet_faces.at(3).row(0) = tet_vertices.row(1);
    tet_faces.at(3).row(1) = tet_vertices.row(2);
    tet_faces.at(3).row(2) = tet_vertices.row(3);

    return tet_faces;
}

// Plane equation function
double TetrahedronCollider::plane_equation(const Eigen::Vector3d& point_pos,
    const Eigen::Vector4d& plane_equations_coeffs)
{
    return plane_equations_coeffs(0) * point_pos(0) +
        plane_equations_coeffs(1) * point_pos(1) + 
        plane_equations_coeffs(2) * point_pos(2) + 
        plane_equations_coeffs(3);
}