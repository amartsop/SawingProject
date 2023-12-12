#pragma once 

#include <armadillo>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <eigen3/Eigen/Dense>

class Utils
{
public:
    Utils() {};

    // Armadillo dvec3 to glm::vec3.
    static glm::vec3 arma_to_glm_vec3(const arma::dvec3& arma_vec);

    // Get the Sign of a number
    template <typename T>
    static int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    // Standard double vector to eigen double vector 
    static Eigen::VectorXd std_dvec_to_eigen_dvec(std::vector<double> vec);

    // Eigen double vector to standard vector
    static std::vector<double> eigen_dvec_to_std_dvec(const Eigen::VectorXd& vec);
};