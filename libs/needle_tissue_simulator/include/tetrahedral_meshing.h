#pragma once

#include <iostream>
#include <filesystem>
#include <ctime>
#include <unistd.h>

#include <eigen3/Eigen/Dense>
#include <igl/decimate.h>
#include <igl/writeSTL.h>
#include "gmsh.h"

class TetrahedralMeshing
{
public:

    // Tetrahedral meshing with decimation
    TetrahedralMeshing(const Eigen::MatrixXd& visual_vertices,
        const Eigen::MatrixXi& visual_faces, bool decimation,
        double decimation_coeff=1.0);

    // Destructor
    ~TetrahedralMeshing();
        
    // Get tetrahedral vertices
    Eigen::MatrixXd get_tetrahedral_vertices(void) { return m_tet_vertices; }

    // Get tetrahedral elements
    Eigen::MatrixXi get_tetrahedral_elements(void) { return m_tet_elements; }

    // Get surface vertices
    Eigen::MatrixXd get_surface_vertices(void) { return m_surf_vertices; }

    // Get surface faces
    Eigen::MatrixXi get_surface_faces(void) { return m_surf_faces; }

private:

    // Tetrahedral vertices (TV - #V by 3 vertex position list)
    Eigen::MatrixXd m_tet_vertices;

    // Tetrahedral faces (TT - #T by 4 list of tet face indices)
    Eigen::MatrixXi m_tet_elements;

    // Surface vertices
    Eigen::MatrixXd m_surf_vertices;

    // Surfaces faces
    Eigen::MatrixXi m_surf_faces;

    // Tetrahedralize stl file
    void tetrahedralize(const std::string& stl_filename, Eigen::MatrixXd& v, 
        Eigen::MatrixXi& e);

    // Extract elements
    Eigen::MatrixXi extract_elements(int element_type,
        int nodes_per_element, const std::vector<std::vector<std::size_t>>&
        element_node_tags);


private:
    // Generate filepath for temporary stl file
    std::string generate_stl_filename(const std::string& abs_path);

    // Generate random string
    std::string generate_random_string(const int len);

    // Relative file extension
    std::string m_rel_tmp_path = "objects/tmp_files/";

    // Absolute file extension
    std::string m_abs_tmp_path;
        
    // STL filename
    std::string m_stl_filename;
};