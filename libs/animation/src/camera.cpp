#include "camera.h"

Camera::Camera(GLFWwindow* window, const glm::vec3& pos, float fov,
    float aspect, float z_near, float z_far)
{
    // Window handler 
    m_window = window;

    // Initialize projection
    m_projection = glm::perspective(glm::radians(fov), aspect, z_near, z_far);

    // Initialise camera axes
    m_camera_front = m_axis_z;
    m_camera_up = m_axis_y;

    // Initialize camera position
    m_position = pos;

    // Initialize aspect and camera distances
    m_aspect = aspect; m_z_near = z_near; m_z_far = z_far; 

    // Set to the desired state (this skips the input)
    m_camera_front = {0.0227377, -0.96484, -0.261852};
    m_camera_up = {0.00616921, -0.26178, 0.965108};
    m_position = {0.0, 0.45, 0.5};
    m_yaw = 360;
    m_pitch = -55.0;
}

void Camera::update(double delta_time)
{
    // Process input
    process_movements(delta_time, m_camera_front);

    // Camera projection 
    m_projection = glm::perspective(glm::radians(m_fov), m_aspect, m_z_near, m_z_far);

    // Aling camera frame to object desired frame
    glm::mat4 rot1 = glm::rotate(glm::radians(90.0f), m_axis_y);
    glm::mat4 rot2 = glm::rotate(glm::radians(90.0f), m_axis_z);
    glm::mat4 rot3 = glm::rotate(glm::radians(-m_yaw), m_axis_y);
    glm::mat4 rot4 = glm::rotate(glm::radians(-m_pitch), m_axis_x);

    glm::mat4 rot = rot1 * rot2 * rot3 * rot4;
    glm::vec3 camera_front = rot * glm::vec4(m_axis_z, 1.0f);
    glm::vec3 camera_up = rot * glm::vec4(m_axis_y, 1.0f);

    // Update camera front and up axis
    m_camera_front = glm::vec3(camera_front);
    m_camera_up = glm::vec3(camera_up);

    // // Print camera state
    // print_camera_state();

    // Process home buttons
    process_home_buttons();

    // Camera view
    m_view = glm::lookAt(m_position, m_position + m_camera_front, m_camera_up);
}


void Camera::process_mouse_input(float x_cursor_offset, float y_cursor_offset)
{
    // Update yaw and pitch
    m_yaw += x_cursor_offset; m_pitch += y_cursor_offset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (m_pitch > 89.0f) m_pitch = 89.0f;
    if (m_pitch < -89.0f) m_pitch = -89.0f;
}


void Camera::process_movements(double delta_time, const glm::vec3& camera_front)
{
    float camera_speed = m_camera_speed_scale * (float)delta_time;

    if (glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS)
    {
        m_position += camera_speed * camera_front;
    }

    if (glfwGetKey(m_window, GLFW_KEY_S) == GLFW_PRESS)
    {
        m_position -= camera_speed * camera_front;
    }

    if (glfwGetKey(m_window, GLFW_KEY_A) == GLFW_PRESS)
    {
        m_position -= glm::normalize(glm::cross(camera_front, m_camera_up)) *
            camera_speed;
    }

    if (glfwGetKey(m_window, GLFW_KEY_D) == GLFW_PRESS)
    {
        m_position += glm::normalize(glm::cross(camera_front, m_camera_up)) *
            camera_speed;
    }

    
}

void Camera::process_mouse_wheel(float y_wheel)
{
    m_fov -= y_wheel;

    if (m_fov < 1.0f) m_fov = 1.0f;
    if (m_fov > 45.0f) m_fov = 45.0f;
}

void Camera::process_home_buttons(void)
{
    if (glfwGetKey(m_window, GLFW_KEY_Y) == GLFW_PRESS)
    {
        m_camera_front = {0.0f, -1.0f, 0.0f};
        m_camera_up = {0.0f, 0.0, 1.0f};
        m_position = {0.0f, 0.5f, 0.0f};
        m_yaw = 90.0;
        m_pitch = 0.0;
    }

    if (glfwGetKey(m_window, GLFW_KEY_X) == GLFW_PRESS)
    {
        m_camera_front = {-1.0f, 0.0f, 0.0f};
        m_camera_up = {0.0f, 0.0, 1.0f};
        m_position = {0.5f, 0.0f, 0.0f};
        m_yaw = 180.0;
        m_pitch = 0.0;
    }

    if (glfwGetKey(m_window, GLFW_KEY_Z) == GLFW_PRESS)
    {
        m_camera_front = {0.0f, 0.0f, -1.0f};
        m_camera_up = {1.0f, 0.0, 0.0f};
        m_position = {0.0f, 0.0f, 0.5f};
        m_yaw = 360.0;
        m_pitch = -89.0;
    }

    if (glfwGetKey(m_window, GLFW_KEY_H) == GLFW_PRESS)
        {
            m_camera_front = {0.8f, -0.5f, -0.2f};
            m_camera_up = {0.2f, -0.1f, 1.0f};
            m_position = {-0.5f, 0.4f, 0.3f};
            m_yaw = 30.0;
            m_pitch = -15.0;
    }

    if (glfwGetKey(m_window, GLFW_KEY_C) == GLFW_PRESS)
    {
        // Set to the desired state (this skips the input)
        m_position = {0.0, 0.45, 0.5};
        m_yaw = 360;
        m_pitch = -45.0;
    }
}


void Camera::print_camera_state(void)
{
    std::cout << "position: " << m_position.x << ", " << m_position.y << ", " << m_position.z << std::endl;
    std::cout << "camera front: " << m_camera_front.x << ", " << m_camera_front.y << ", " << m_camera_front.z << std::endl;
    std::cout << "camera up: " << m_camera_up.x << ", " << m_camera_up.y << ", " << m_camera_up.z << std::endl;
    std::cout << "yaw: " << m_yaw << std::endl;
    std::cout << "pitch: " << m_pitch << std::endl;
}