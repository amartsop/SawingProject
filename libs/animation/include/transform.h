#pragma once 

#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>

class Transform
{
    public:
        Transform(const glm::vec3& pos = glm::vec3(0.0f, 0.0f, 0.0f), 
            const glm::vec3& rot = glm::vec3(0.0f, 0.0f, 0.0f), 
            const glm::vec3& scale = glm::vec3(1.0f, 1.0f, 1.0f));

        //Getters
        inline glm::vec3 getPos() const { return m_pos; }
        inline glm::vec3 getRot() const { return m_rot; }
        inline glm::vec3 getScale() const { return m_scale; }
        glm::mat4 getModel(void);

        //Setters
        inline void setPos(const glm::vec3& pos) { m_pos = pos; }
        inline void setRot(const glm::vec3& rot) { m_rot = rot; }
        inline void setScale(const glm::vec3& scale) { m_scale = scale; }

    private:

        // Update model transformation matrix
        void updateModel(void);

        // Object position
        glm::vec3 m_pos;

        // Object orientation euler angles
        glm::vec3 m_rot;
    
        // Object scale
        glm::vec3 m_scale;

        // Object model matrix
        glm::mat4 m_model = glm::mat4(1.0f);
};
