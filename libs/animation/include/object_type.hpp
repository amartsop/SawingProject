#pragma once


#define GLM_ENABLE_EXPERIMENTAL
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <indexed_model.h>

namespace obj_type
{
    // Imported surface properties
    struct PointProperties{

        // Identifier
        unsigned int id;

        // Object position
        glm::vec3 pos;

        // Object color
        glm::vec3 color=glm::vec3({0.0f, 0.0f, 0.0f});

        // Object size
        double size=0.01;

        // Material shininess 
        float shininess = 64.0f;

        // Material shininess 
        glm::vec3 specular = glm::vec3(0.5f, 0.5f, 0.5f);

        // Wireframe flag 
        bool wireframe = false;
        
        // Draw type
        GLenum draw_type = GL_STATIC_DRAW;
    };

    // Line properties
    struct LineProperties{

        // Identifier
        unsigned int id;

        // Point 1
        glm::vec3 p1;

        // Point 2
        glm::vec3 p2;

        // Object color
        glm::vec3 color=glm::vec3({0.0f, 0.0f, 0.0f});

        // Thickness
        double thickness = 0.2;

        // Material shininess 
        float shininess = 64.0f;

        // Material shininess 
        glm::vec3 specular = glm::vec3(0.5f, 0.5f, 0.5f);

        // Wireframe flag 
        bool wireframe = false;

        // Draw type
        GLenum draw_type = GL_STATIC_DRAW;
    };

    // Box properties
    struct BoxProperties{

        // Identifier
        unsigned int id;

        // Object position
        glm::vec3 pos;

        // Object orientation
        glm::vec3 euler;

        // Object dimensions
        glm::vec3 dimensions=glm::vec3({1.0f, 1.0f, 1.0f});

        // Object color
        glm::vec3 color=glm::vec3({0.0f, 0.0f, 0.0f});

        // Material shininess 
        float shininess = 64.0f;

        // Material shininess 
        glm::vec3 specular = glm::vec3(0.5f, 0.5f, 0.5f);

        // Wireframe flag 
        bool wireframe = false;

        // Draw type
        GLenum draw_type = GL_STATIC_DRAW;
    };

    // Plane properties
    struct PlaneProperties{

        // Identifier
        unsigned int id;

        // Object position
        glm::vec3 pos;

        // Object orientation
        glm::vec3 euler;

        // Object dimensions
        glm::vec3 dimensions=glm::vec3({1.0f, 1.0f, 1.0f});

        // Object color
        glm::vec3 color=glm::vec3({0.0f, 0.0f, 0.0f});

        // Material shininess 
        float shininess = 64.0f;

        // Material shininess 
        glm::vec3 specular = glm::vec3(0.5f, 0.5f, 0.5f);

        // Wireframe flag 
        bool wireframe = false;

        // Draw type
        GLenum draw_type = GL_STATIC_DRAW;
    };

    // Sphere properties
    struct SphereProperties{

        // Identifier
        unsigned int id;

        // Object position
        glm::vec3 pos;

        // Object dimensions
        double radius;

        // Object color
        glm::vec3 color=glm::vec3({0.0f, 0.0f, 0.0f});

        // Material shininess 
        float shininess = 64.0f;

        // Material shininess 
        glm::vec3 specular = glm::vec3(0.5f, 0.5f, 0.5f);

        // Wireframe flag 
        bool wireframe = false;

        // Draw type
        GLenum draw_type = GL_STATIC_DRAW;
    };


    // Imported surface properties
    struct ImportedSurfaceProperties{

        // Identifier
        unsigned int id;

        // Object position
        glm::vec3 pos;

        // Object orientation
        glm::vec3 euler;

        // Object scale
        glm::vec3 scale=glm::vec3({1.0f, 1.0f, 1.0f});

        // Object color
        glm::vec3 color=glm::vec3({0.0f, 0.0f, 0.0f});

        // Material shininess 
        float shininess = 64.0f;

        // Material shininess 
        glm::vec3 specular = glm::vec3(0.5f, 0.5f, 0.5f);

        // Wireframe flag 
        bool wireframe = false;

        // Shader filename
        std::string shader_filename;

        // Texture filename
        std::string texture_filename;

        // Geometry
        std::string mesh_filename;

        // Draw type
        GLenum draw_type = GL_STATIC_DRAW;
    };

    // Indexed surface properties
    struct IndexedSurfaceProperties{

        // Identifier
        unsigned int id;

        // Object position
        glm::vec3 pos;

        // Object orientation
        glm::vec3 euler;

        // Object scale
        glm::vec3 scale=glm::vec3({1.0f, 1.0f, 1.0f});

        // Object color
        glm::vec3 color=glm::vec3({0.0f, 0.0f, 0.0f});

        // Material shininess 
        float shininess = 64.0f;

        // Material shininess 
        glm::vec3 specular = glm::vec3(0.5f, 0.5f, 0.5f);

        // Wireframe flag 
        bool wireframe = false;

        // Shader filename
        std::string shader_filename;

        // Texture filename
        std::string texture_filename;

        // Geometry
        IndexedModel indexed_model;

        // Draw type
        GLenum draw_type = GL_STATIC_DRAW;
    };
}


