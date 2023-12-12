#include "../include/friction_model.h"

FrictionModel::FrictionModel(/* args */)
{
    // Initialize negative and positive dynamic friction coeffs
    m_cn = - 1.2 * 1.e-1; m_cp = 1. * 1.e-1;
    
    // Initialize negative and positive damping coeffs 
    m_bn = - 1.e0; m_bp = 4.e-1;

    // Initialize negative and positive static friction coeffs
    m_dn = - 0. * 1.e-4; m_dp = 0 * 1.e-4;

    // Initialize velocity bound
    m_dv = 5.e-4;
}


// Calculate friction force
double FrictionModel::calculate_friction_force(double vel_insertion_dir)
{
    // Initialize friction force
    double fr = 0.0;

    // Check conditions
    if (vel_insertion_dir <= - m_dv / 2.0)
    {
        fr = m_cn * utils::sign_func(vel_insertion_dir) + m_bn * vel_insertion_dir;
    }

    if (vel_insertion_dir > - m_dv/2 && vel_insertion_dir <= 0.0)
    {
        fr = m_dn;
    }

    if (vel_insertion_dir > 0 && vel_insertion_dir < m_dv / 2.)
    {
        fr = m_dp;
    }

    if (vel_insertion_dir >= m_dv / 2.0)
    {
        fr = m_cp * utils::sign_func(vel_insertion_dir) + m_bp * vel_insertion_dir;
    }
    

    return -fr;
}