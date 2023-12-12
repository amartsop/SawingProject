#pragma once

#include <iostream>
#include <glad/glad.h>

#include "stb_image.h"
#include <cassert>

class Texture{

    public:

        Texture(){};

        // Assign textures
        void assign(const std::string& filename);

        // Bind textures
        void bind(unsigned int unit);
    
        GLuint get_ID(void) const { return m_texture; }

        virtual ~Texture();

    private:
        GLuint m_texture;
};