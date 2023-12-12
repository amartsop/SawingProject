#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <armadillo>
#include <eigen3/Eigen/Dense>
#include <filesystem>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <scene.hpp>
#include <object_type.hpp>
#include <object.h>
#include <imported_surface.h>
#include <indexed_surface.h>
#include <primitives/point.h>
#include <primitives/line.h>
#include <primitives/box.h>

#include "utils.h"
// #include <user_interface.h>
// #include <world_interface.h>
// #include <igl/readOBJ.h>


class AnimationInterface
{

public:
    AnimationInterface(){};

    // Initialize animation
    void initialize_animation(Scene<AnimationInterface>* scene);

    // Update objects
    void update(double real_time);

    // // Set world interface
    // inline void set_world_interface(std::shared_ptr<WorldInterface>&
    //     world_interface) { m_world_interface = world_interface; }

private:
    /************************** Objects *******************/
    // Objects container
    std::vector<std::shared_ptr<Object>> m_objects;

    // Simple shader path
    std::string m_simple_shader_path;

    // Current path
    std::filesystem::path m_current_path;

private:

    // Generate background
    void generate_background(void);

    // Background id
    int m_background_id;

    // Background relative mesh name
    std::filesystem::path m_bkg_rel_mesh_name =
        "objects/checker_floor/checker_floor.obj";

    // Background relative texture names
    std::filesystem::path m_bkg_rel_texture_name =
        "objects/checker_floor/checker_floor.png";

private:

    // Generate sawing platform
    void generate_sawing_platform(void);

    // Sawing platform id
    int m_sawing_platform_id;

    // Sawing platform relative mesh name
    std::filesystem::path m_sp_rel_mesh_name =
        "objects/sawing_platform/sawing_platform.obj";

    // Sawing platform relative texture names
    std::filesystem::path m_sp_rel_texture_name =
        "objects/sawing_platform/sawing_platform.png";

private:

    // Generate target object
    void generate_target_object(void);

    // Target object id
    int m_target_object_id;

    // Sawing platform relative mesh name
    std::filesystem::path m_to_rel_mesh_name =
        "objects/cadillac/cadillac.obj";

    // Sawing platform relative texture names
    std::filesystem::path m_to_rel_texture_name =
        "objects/cadillac/cadillac.png";

// private:
//     // World interface pointer
//     std::shared_ptr<WorldInterface> m_world_interface;

};