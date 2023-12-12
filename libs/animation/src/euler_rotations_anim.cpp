#include "../include/euler_rotations_anim.h"

// Basic rotation matrix wrt x axis
arma::dmat33 EulerRotationsAnim::basic_rotation_x(double x)
{
    double cx = cos(x); double sx = sin(x);
    return { {1.0f, 0.0f, 0.0f}, {0.0f, cx, -sx}, {0.0f, sx, cx}};
}

// Basic rotation matrix wrt y axis
arma::dmat33 EulerRotationsAnim::basic_rotation_y(double x)
{
    double cx = cos(x); double sx = sin(x);
    return {{cx, 0.0f, sx}, {0.0f, 1.0f, 0.0f}, {-sx, 0.0f, cx}};
}

// Basic rotation matrix wrt z axis
arma::dmat33 EulerRotationsAnim::basic_rotation_z(double x)
{
    double cx = cos(x); double sx = sin(x);
    return {{cx, -sx, 0.0f}, {sx, cx, 0.0f}, {0.0f, 0.0f, 1.0f}};
}

// Euler rotation matrix z-y'-x''
arma::dmat33 EulerRotationsAnim::rotation(double phi, double theta, double psi)
{
    return (basic_rotation_z(psi) * basic_rotation_y(theta) *
        basic_rotation_x(phi));
}

arma::dmat33 EulerRotationsAnim::rotation(const arma::dvec& euler_angles)
{
    return rotation(euler_angles(0), euler_angles(1), euler_angles(2));
}

// Rotation matrix to euler angles
arma::dvec EulerRotationsAnim::rotation_to_euler(const arma::dmat& rot_mat)
{
    // Rotation matrix components
    double r11 = rot_mat(0, 0);
    double r12 = rot_mat(0, 1);
    double r13 = rot_mat(0, 2);

    double r21 = rot_mat(1, 0);

    double r31 = rot_mat(2, 0);
    double r32 = rot_mat(2, 1);
    double r33 = rot_mat(2, 2);

    // Initialize euler angles
    double phi, theta, psi;

    if (r31 == 1 || r31 == -1) {
        // Set psi aribtrarily
        psi = 0;

        if(r31 == - 1) { theta = M_PI_2; phi = psi + atan2(r12, r13); }
        else {theta = - M_PI_2, phi = -psi + atan2(-r12, -r13); }
    }
    else {
        theta = - asin(r31);
        phi = atan2( (r32 / cos(theta)), (r33 / cos(theta)) );
        psi = atan2( (r21 / cos(theta)), (r11 / cos(theta)) );
    }

    return {phi, theta, psi};
}