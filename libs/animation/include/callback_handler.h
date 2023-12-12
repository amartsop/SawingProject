#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <memory>

#include "camera.h"
#include "display.h"

class CallbackHandler
{
    public:
        CallbackHandler(GLFWwindow* window);

        // Cursor position callback function 
        void mouse_cursor_callback(double x_pos, double y_pos);
        
        // Mouse scroll callback function 
        void mouse_wheel_callback(float wheel_x, float wheel_y);

        // Set cursor initial coordinates 
        void set_cursor_coords(float cursor_x, float cursor_y);

        // Set camera inputs 
        void set_camera(std::shared_ptr<Camera> camera) { m_camera = camera; }

        // Mouse button callback function
        void mouse_button_callback(int button, int action, int mods);

    private:
        // Camera handle 
        std::shared_ptr<Camera> m_camera;

        // Mouse movement variables
        float m_sensitivity = 0.07f;
        bool m_first_mouse = true;
        float m_last_x, m_last_y;

        // Middle mouse status
        bool m_middle_mouse_down = false;

    public:
    
        // Frame buffer size callback
        void frame_buffer_size_callback(int widht, int height);

        // Set camera inputs 
        void set_display(Display *display) { m_display = display; }

    private:

        // Display handle
        Display* m_display; 

};

