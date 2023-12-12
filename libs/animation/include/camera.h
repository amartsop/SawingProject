#pragma once

#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include <GLFW/glfw3.h>



class Camera
{
    public:
        //Camera constructor
        Camera(GLFWwindow* window, const glm::vec3& pos, float fov,
            float aspect, float z_near, float z_far);

        void update(double delta_time);

        glm::mat4 get_projection(void) const { return m_projection; }
        glm::mat4 get_view(void) const { return m_view; }
        glm::vec3 get_camera_position(void) const { return m_position; }

        void process_mouse_input(float x_cursor_offset, float y_cursor_offset);

        void process_mouse_wheel(float y_wheel);

    private:
        GLFWwindow* m_window;

        // Process movements
        void process_movements(double delta_time, const glm::vec3& camera_front);

        // Home buttons
        void process_home_buttons(void);

        // World axes
        const glm::vec3 m_axis_x = glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::vec3 m_axis_y = glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 m_axis_z = glm::vec3(0.0f, 0.0f, 1.0f);
        
        // Camera axes
        glm::vec3 m_camera_up, m_camera_front;

        // Camera speed scale
        const float m_camera_speed_scale = 0.3f;

        // Camera aspect and depth parameres 
        float m_aspect, m_z_near, m_z_far;

        // Camera view and projection
        glm::mat4 m_projection, m_view;

        // Camera position 
        glm::vec3 m_position;

        // Camera pitch and yaw
        float m_pitch =  -0.4f; float m_yaw = 45.0f;

        // Camera field of view
        float m_fov = 45.0f;

        // Print camera state
        void print_camera_state(void);
};