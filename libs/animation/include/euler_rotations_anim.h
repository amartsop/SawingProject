#pragma once 

#include <iostream>
#include <armadillo>
#include <math.h>

class EulerRotationsAnim
{
public:
    // Basic rotation matrix wrt x axis
    static arma::dmat33 basic_rotation_x(double x);
    
    // Basic rotation matrix wrt y axis
    static arma::dmat33 basic_rotation_y(double x);

    // Basic rotation matrix wrt z axis
    static arma::dmat33 basic_rotation_z(double x);

    // Euler rotation matrix z-y'-x''
    static arma::dmat33 rotation(double phi, double theta, double psi);
    static arma::dmat33 rotation(const arma::dvec& euler_angles);

    // Rotation matrix to euler angles
    static arma::dvec rotation_to_euler(const arma::dmat& rot_mat);
};
