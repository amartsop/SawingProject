#pragma once 

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <iostream>
#include <fstream>
#include <sstream>


class Shader
{
    public:
        // Constructor
        Shader(){};

        // Initialize shader
        void initialize(const std::string& filename);

        // Bind shader 
        void bind();

        // Destructor
        virtual ~Shader();

        // Uniform setters
        void set_bool(const std::string& name, bool value) const;
        void set_int(const std::string& name, int value) const;
        void set_float(const std::string& name, float value) const;
        void set_vec2(const std::string& name, const glm::vec2 &value) const;
        void set_vec2(const std::string& name, float x, float y) const;
        void set_vec3(const std::string& name, const glm::vec3 &value) const;
        void set_vec3(const std::string& name, float x, float y, float z) const;
        void set_vec4(const std::string& name, const glm::vec4 &value) const;
        void set_vec4(const std::string& name, float x, float y, float z,
            float w) const;
        void set_mat2(const std::string& name, const glm::mat2 &mat) const;
        void set_mat3(const std::string& name, const glm::mat3 &mat) const;
        void set_mat4(const std::string& name, const glm::mat4 &mat) const;

    private:
        static std::string load_shader(const std::string& filename);

        static GLuint compiler_shader(const std::string& shader_source,
            GLuint shader_type);

        static void check_shader_error(GLuint shader, GLuint flag,
            bool is_program, const std::string& error_message);

    //Emumeations of shaders
    enum{
        VERTEX_SHADER,
        FRAGMENT_SHADER,
        NUM_SHADERS
    };

    // Shaders type
    const GLuint m_shaders_type[NUM_SHADERS] = {GL_VERTEX_SHADER,
        GL_FRAGMENT_SHADER};


    // Shaders variables 
    GLuint m_program;
    std::string m_shaders_source[NUM_SHADERS];
    GLuint m_shaders[NUM_SHADERS];

};
