#include "../include/tetrahedral_meshing.h"

// Tetrahedral meshing with decimation
TetrahedralMeshing::TetrahedralMeshing(const Eigen::MatrixXd& visual_vertices,
    const Eigen::MatrixXi& visual_faces, bool decimation,
    double decimation_coeff)
{
    if (decimation)
    {
        // Initialize j_vec
        Eigen::VectorXi j_vec;
    
        // Desired faces num
        int des_faces = (int) (decimation_coeff * visual_faces.rows());
    
        // Decimate mesh
        igl::decimate(visual_vertices, visual_faces, des_faces,
            m_surf_vertices, m_surf_faces, j_vec);
    }
    else 
    {
        m_surf_vertices = visual_vertices;
        m_surf_faces = visual_faces;
    }
    
    // Create absolute filepath for stl file
    m_abs_tmp_path = (std::filesystem::current_path() / m_rel_tmp_path).string();

    // Generate stl filename
    m_stl_filename = generate_stl_filename(m_abs_tmp_path);

    // Generate stl file
    igl::writeSTL(m_stl_filename, m_surf_vertices, m_surf_faces);
    
    // Tetrahedralize
    tetrahedralize(m_stl_filename, m_tet_vertices, m_tet_elements);
}

// Tetrahedralize stl file
void TetrahedralMeshing::tetrahedralize(const std::string& stl_filename,
    Eigen::MatrixXd& v, Eigen::MatrixXi& e)
{
    // Tetrahedron id and number of nodes per tetrahedron (GMSH constants)
    const int tet_id = 4;
    const int tet_nodes = 4;

    // Initialize gmsh
    gmsh::initialize();

    // Add model
    gmsh::model::add("t1");

    // Read stl file
    gmsh::merge(stl_filename);

   // Create a volume from all the surfaces
    std::vector<std::pair<int, int> > s;
    gmsh::model::getEntities(s, 2);
    std::vector<int> sl;
    for(auto surf : s) sl.push_back(surf.second);
    int l = gmsh::model::geo::addSurfaceLoop(sl);
    gmsh::model::geo::addVolume({l});
    gmsh::model::geo::synchronize();
    
    // Generate mesh
    gmsh::model::mesh::generate(3);

    /*********************** Get mesh properties **************************/
    /***************** Nodes *********************/
    // Initialize node vectors
    std::vector<size_t> node_tags;
    std::vector<double> node_coords;
    std::vector<double> parametric_coords;

    // Get node properties
    gmsh::model::mesh::getNodes(node_tags, node_coords, parametric_coords, -1,
        -1, false, false);

    // Number of nodes
    size_t vertices_num = node_coords.size() / 3;

    // Initialize vertices
    v = Eigen::MatrixXd(vertices_num, 3);

    // Vecrtices counter
    size_t vertices_counter = 0;

    // Get node coordinates
    for(size_t i = 0; i < node_coords.size(); i=i+3)
    {
        // Generate vertices
        v(vertices_counter, 0) = node_coords.at(i);
        v(vertices_counter, 1) = node_coords.at(i+1);
        v(vertices_counter, 2) = node_coords.at(i+2);

        // Update vertices counter
        vertices_counter++;
    }

    /***************** Elements *********************/
    // Get element properties
    std::vector<int> element_types;
    std::vector<std::vector<std::size_t>> element_tags;
    std::vector<std::vector<std::size_t>> element_node_tags;

    gmsh::model::mesh::getElements(element_types, element_tags, element_node_tags);

    // Get the index of 2D triangles and boundary lines
    int tet_index = 0;
    for (size_t i = 0; i < element_types.size(); i++)
    {
        if (element_types.at(i) == tet_id){ tet_index = i; }
    }

    // Extract tet elements
    e = extract_elements(tet_index, tet_nodes, element_node_tags);
}

// Extract elements
Eigen::MatrixXi TetrahedralMeshing::extract_elements(int element_type,
    int nodes_per_element, const std::vector<std::vector<std::size_t>>&
    element_node_tags)
{
    // Number of elements    
    size_t elements_num = element_node_tags.at(element_type).size() / nodes_per_element;

    // Elements counter
    size_t elements_counter = 0;

    // Initialize faces
    Eigen::MatrixXi elements = Eigen::MatrixXi(elements_num, nodes_per_element);

    // Element node tags
    for (size_t i = 0; i < element_node_tags.at(element_type).size(); i=i+nodes_per_element)
    {
        for (size_t j = 0; j < nodes_per_element; j++)
        {
            elements(elements_counter, j) =
                element_node_tags.at(element_type).at(i+j) - 1;
        }
        // Update element scounter
        elements_counter++;
    }

    return elements;
}


// Generate filepath for temporary stl file
std::string TetrahedralMeshing::generate_stl_filename(const std::string& abs_path)
{
    // Create directory
    std::filesystem::create_directory(abs_path);
    
    // Stl abs filename
    std::string stl_filename;
    
    // File exists flag
    bool file_exists = true;
    while(file_exists)
    {
        // Generate random filename
        srand((unsigned)time(NULL) * getpid());
        std::string mesh_filename = generate_random_string(5) + ".stl";

        // Generate absolute tmp filename
        stl_filename = abs_path + mesh_filename;
    
        // Update file exists flag
        file_exists = std::filesystem::exists(stl_filename);
    }

    return stl_filename;
}

// Generate random string
std::string TetrahedralMeshing::generate_random_string(const int len) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    std::string tmp_s;
    tmp_s.reserve(len);

    for (int i = 0; i < len; ++i) {
        tmp_s += alphanum[rand() % (sizeof(alphanum) - 1)];
    }
    
    return tmp_s;
}

TetrahedralMeshing::~TetrahedralMeshing()
{
    // Remove tmp directory
    std::filesystem::remove_all(m_abs_tmp_path);
}