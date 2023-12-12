#include "../include/utils.h"

// Armadillo dvec3 to glm::vec3
glm::vec3 Utils::arma_to_glm_vec3(const arma::dvec3& arma_vec)
{
    return glm::vec3({arma_vec(0), arma_vec(1), arma_vec(2)});
}
// Standard double vector to eigen double vector 
Eigen::VectorXd Utils::std_dvec_to_eigen_dvec(std::vector<double> vec)
{
    Eigen::Map<Eigen::VectorXd> eigen_vec(&vec[0], vec.size());
    return eigen_vec;
}

// Eigen double vector to standard vector
std::vector<double> Utils::eigen_dvec_to_std_dvec(const Eigen::VectorXd& vec)
{
    std::vector<double> std_vec(vec.data(), vec.data() + vec.rows() * vec.cols());
    return std_vec;
}
