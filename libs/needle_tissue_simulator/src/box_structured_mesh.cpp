#include "../include/box_structured_mesh.h"

BoxStructuredMesh::BoxStructuredMesh(double width, double height, double depth,
    const Eigen::Vector3i& mesh_resolution, const Eigen::Matrix3d& rot_mat,
    const Eigen::Vector3d t_vec)
{
    // Generate volume points
    m_pts = generate_volume_points(width, height, depth);

    // Generate sturctured mesh
    m_box_mesh = generate_structured_mesh(m_pts, mesh_resolution);

    // Clean up box surface
    int max_vertex_idx = m_box_surf.faces.maxCoeff();
    Eigen::MatrixXd sub_mat = m_box_surf.vertices(Eigen::seq(0, max_vertex_idx),
        Eigen::all);
    m_box_surf.vertices = sub_mat;

    // Transform all vertices
    for (size_t i = 0; i < m_box_mesh.vertices.rows(); i++)
    {
        m_box_mesh.vertices.row(i) =
            (t_vec + rot_mat * m_box_mesh.vertices.row(i).transpose()).transpose();
    }

    for (size_t i = 0; i < m_box_surf.vertices.rows(); i++)
    {
        m_box_surf.vertices.row(i) =
            (t_vec + rot_mat * m_box_surf.vertices.row(i).transpose()).transpose();
    }
}

// Generate structured mesh
Mesh3D::TetMesh BoxStructuredMesh::generate_structured_mesh(const
    Eigen::MatrixXd& edge_pts, const Eigen::VectorXi& mesh_resolution)
{
    // Initialize box mesh
    Mesh3D::TetMesh box_mesh;

    // Initialize mesh
    gmsh::initialize();

    gmsh::model::add("t1");

    // Tolerance
    double lc = 1e-2;

    // Generate points
    int p[8];

    for (int i = 0; i < 8; i++)
    {
        p[i] = gmsh::model::geo::addPoint(edge_pts(i, 0), edge_pts(i, 1),
            edge_pts(i, 2), lc);
    }

    // Add lines
    int l1 = gmsh::model::geo::addLine(p[0], p[1]);
    int l2 = gmsh::model::geo::addLine(p[1], p[2]);
    int l3 = gmsh::model::geo::addLine(p[2], p[3]);
    int l4 = gmsh::model::geo::addLine(p[3], p[0]);

    // Add curve loops
    std::vector<int> lines = {l1, l2, l3, l4};
    int curve = gmsh::model::geo::addCurveLoop(lines);

    // Add surface
    int surface = gmsh::model::geo::addPlaneSurface({curve});

    // Set spacing
    gmsh::model::geo::mesh::setTransfiniteCurve(l1, mesh_resolution(0));
    gmsh::model::geo::mesh::setTransfiniteCurve(l2, mesh_resolution(0));
    gmsh::model::geo::mesh::setTransfiniteCurve(l3, mesh_resolution(1));
    gmsh::model::geo::mesh::setTransfiniteCurve(l4, mesh_resolution(1));

    gmsh::model::geo::mesh::setTransfiniteSurface(surface, "Left", lines);

    // Smoothing
    gmsh::option::setNumber("Mesh.Smoothing", 100);

    double h = - abs(edge_pts(0, 2) - edge_pts(7, 2));
    std::vector<std::pair<int, int> > ov;
    gmsh::model::geo::extrude({{2, surface}}, 0, 0, h, ov, {mesh_resolution(2)});

    gmsh::model::geo::synchronize();
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
    box_mesh.vertices = Eigen::MatrixXd(vertices_num, 3);

    // Vecrtices counter
    size_t vertices_counter = 0;

    // Get node coordinates
    for(size_t i = 0; i < node_coords.size(); i=i+3)
    {
        // Generate vertices
        box_mesh.vertices(vertices_counter, 0) = node_coords.at(i);
        box_mesh.vertices(vertices_counter, 1) = node_coords.at(i+1);
        box_mesh.vertices(vertices_counter, 2) = node_coords.at(i+2);

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
    int triangle_index = 0;
    for (size_t i = 0; i < element_types.size(); i++)
    {
        if (element_types.at(i) == m_tet_id){ tet_index = i; }
        if (element_types.at(i) == m_triangle_id){ triangle_index = i; }
    }
        
    // Extract tet elements
    box_mesh.elements = extract_elements(tet_index, m_tet_nodes, element_node_tags);
        
    // Extract triangular elements
    m_box_surf.faces = extract_elements(triangle_index, m_triangle_nodes,
        element_node_tags);
    m_box_surf.vertices = box_mesh.vertices;

    return box_mesh;
}

// Extract elements
Eigen::MatrixXi BoxStructuredMesh::extract_elements(int element_type,
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


// Generate volume points
Eigen::MatrixXd BoxStructuredMesh::generate_volume_points(double width,
    double height, double depth)
{
    // Define half distances
    double w2 = width / 2.0;
    double h2 = height / 2.0;
    double d2 = depth / 2.0;

    // Initialize points container
    Eigen::MatrixXd pts = Eigen::MatrixXd::Zero(8, 3);

    // Points
    pts.row(0) << -w2, -h2, d2; pts.row(1) << w2, -h2, d2;
    pts.row(2) << w2, h2, d2; pts.row(3) << -w2, h2, d2;
    pts.row(4) << w2, -h2, -d2; pts.row(5) << w2, h2, -d2;
    pts.row(6) << -w2, h2, -d2; pts.row(7) << -w2, -h2, -d2;

    return pts;
}