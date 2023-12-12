#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

class Display
{
    public:

        Display(const char* title, int width=1920, int height=1080);

        // Clear screen
        void clear(float r=0.0f, float g=0.0f, float b=0.0f, float a=1.0f);

        bool is_closed(void);

        // Update display
        void update(void); 

        // Get window handle
        GLFWwindow* get_window_handle(void) const { return m_window; }

        // Get window width and height
        int get_window_width() const { return m_width; }
        int get_window_height() const { return m_height; }

        // Set window status
        void set_status(const bool& close) { m_is_closed = close; }

        // Process frame buffer size (callback)
        void process_frame_buffer_size(int width, int height);

        virtual ~Display();

    private:

        // Window handle
        GLFWwindow* m_window;

        // Window state
        bool m_is_closed;

        // Window settings
        int m_width, m_height;

        /****************** Callback functions ********************************/
        // Change window size callback
        static void frame_buffer_size_callback(GLFWwindow* window, int width,
            int height);
};
