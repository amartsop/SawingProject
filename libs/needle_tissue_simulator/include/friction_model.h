#pragma once 

#include <iostream>
#include "utils.h"
#include <eigen3/Eigen/Dense>

class FrictionModel
{
public:

    FrictionModel(/* args */);
    
    // Calculate friction force
    double calculate_friction_force(double vel_insertion_dir);
    
private:
    
    // Negative and positive dynamic friction coeffs (N/m)
    double m_cn, m_cp;
    
    // Negative and positive static friction coeffs (N/m)
    double m_dn, m_dp;
    
    // Negative and positive damping coefficients (Ns/m^2)
    double m_bn, m_bp;
    
    // Velocity bound (m/s)
    double m_dv;
    
};
