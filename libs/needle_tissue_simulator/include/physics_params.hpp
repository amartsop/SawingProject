#pragma once

#include <iostream>

class PhysicsParams
{
public:
    PhysicsParams(){};

    // Gravity constant (m / s^2)
    double grav = 9.81;

    // Needle length
    double needle_length = 2.7e-1; // Needle length (m)
    double needle_radius = 6.35e-4; // Needle radius (m)
    
    // Needle material properties 
    double needle_density = 7850; // Stainless steel density (kg/m^3)
    double needle_young_modulus = 2e11; // Stainless steel Young Modulus (N/m^2)
    double needle_shear_modulus = 7.93e10; // Stainless steel Shear Modulus (N/m^2)

    // Tissue mesh dimensions
    double tissue_depth = 211. / 1000.; // Tissue depth x (m)
    double tissue_width = 141. / 1000.; // Tissue width y (m)
    double tissue_height = 77. / 1000.; // Tissue height z (m)

    // Gelatin density
    double tissue_density = 1060; // Tissue density kg/m3

    // Young modulus (N/m^2)
    // double young_modulus = 1.9 * 1.e3;
    double young_modulus = 2.3 * 1.e3;

    // Poisson ratio
    double v =  0.49;

    // Lame parameters
    double mu = (0.5 * young_modulus) / (1.0 + v);
    double lambda = (young_modulus * v) / ( (1.0 + v) * (1.0 - 2.0 * v));
};