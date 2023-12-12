#include <callback_handler.h>

CallbackHandler::CallbackHandler(GLFWwindow* window)
{
    // Mouse callback function
    glfwSetWindowUserPointer(window, reinterpret_cast<void *>(this));
}


void CallbackHandler::set_cursor_coords(float cursor_x, float cursor_y)
{
    m_last_x = cursor_x; m_last_y = cursor_y;
}


void CallbackHandler::mouse_cursor_callback(double x_pos, double y_pos)
{
    if (m_first_mouse)
    {
        m_last_x = x_pos; m_last_y = y_pos;
        m_first_mouse = false;
    }

    float xoffset = x_pos - m_last_x;
    float yoffset = m_last_y - y_pos; 
    m_last_x = x_pos; m_last_y = y_pos;

    xoffset *= m_sensitivity; yoffset *= m_sensitivity;
    
    if (m_middle_mouse_down)
    {
        m_camera->process_mouse_input(xoffset, yoffset);
    }
}

// Mouse scroll callback function 
void CallbackHandler::mouse_wheel_callback(float wheel_x, float wheel_y)
{
   m_camera->process_mouse_wheel(wheel_y);
}

// Mouse button callback
void CallbackHandler::mouse_button_callback(int button, int action, int mods)
{
   if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS)
   {
       m_middle_mouse_down = true;
   }   
   else
   {
       m_middle_mouse_down = false;
   }   
}

// Frame buffer size callback
void CallbackHandler::frame_buffer_size_callback(int width, int height)
{
    m_display->process_frame_buffer_size(width, height);
}