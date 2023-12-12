#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include "callback_handler.h"

class CallbackSetter
{
    public:
        CallbackSetter(GLFWwindow* window);

    private:
        // Cursor position calllback function
        static void mouse_cursor_callback(GLFWwindow* window, double x_pos,
            double y_pos);

        // Scroll wheel callback function
        static void mouse_wheel_callback(GLFWwindow* window, double x_offset,
            double y_offset);

        // Mouse button callback
        static void mouse_button_callback(GLFWwindow* window,
            int button, int action, int mods);

        // Dispay frame buffer size callback
        static void frame_buffer_size_callback(GLFWwindow* window, int width,
            int height);
};


