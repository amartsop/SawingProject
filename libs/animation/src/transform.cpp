#include <transform.h>

Transform::Transform(const glm::vec3& pos, const glm::vec3& rot,
    const glm::vec3& scale)
{
    // Initialize object state
    m_pos = pos; m_rot = rot; m_scale = scale;

    // Update model 
    updateModel();
}

// Update and return model transformation matrix
glm::mat4 Transform::getModel(void)
{
    updateModel(); return m_model;
 }


// Update model transformation matrix
void Transform::updateModel(void)
{
    glm::mat4 posMatrix = glm::translate(m_pos);
    glm::mat4 rotxMatrix = glm::rotate(m_rot.x, glm::vec3(1.0f, 0.0f, 0.0f));
    glm::mat4 rotyMatrix = glm::rotate(m_rot.y, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 rotzMatrix = glm::rotate(m_rot.z, glm::vec3(0.0f, 0.0f, 1.0f));
    glm::mat4 scaleMatrix = glm::scale(m_scale);
    glm::mat4 rotMatrix  = rotzMatrix * rotyMatrix * rotxMatrix;
    m_model = posMatrix * rotMatrix * scaleMatrix;
}
