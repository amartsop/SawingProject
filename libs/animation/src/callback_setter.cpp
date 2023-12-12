#include <callback_setter.h>

CallbackSetter::CallbackSetter(GLFWwindow* window)
{
    // Mouse cursor position callback 
    glfwSetCursorPosCallback(window, mouse_cursor_callback);
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    // Mouse wheel scroll callback
    glfwSetScrollCallback(window, mouse_wheel_callback);

    // Mouse button callback
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // Frame buffer size callback
    glfwSetFramebufferSizeCallback(window, frame_buffer_size_callback);
}


void CallbackSetter::mouse_cursor_callback(GLFWwindow* window, double x_pos,
    double y_pos)
{
    CallbackHandler *event  = 
        reinterpret_cast<CallbackHandler *>(glfwGetWindowUserPointer(window));

    if(event)
    {
        event->mouse_cursor_callback((float)x_pos, (float)y_pos);
    }
}


void CallbackSetter::mouse_wheel_callback(GLFWwindow* window, double x_offset,
    double y_offset)
{
    CallbackHandler *event  = 
        reinterpret_cast<CallbackHandler *>(glfwGetWindowUserPointer(window));

    if(event)
    {
        event->mouse_wheel_callback((float)x_offset, (float)y_offset);
    }
}


// Mouse button callback
void CallbackSetter::mouse_button_callback(GLFWwindow* window, int button,
    int action, int mods)
{

    CallbackHandler *event  = 
        reinterpret_cast<CallbackHandler *>(glfwGetWindowUserPointer(window));

    if(event)
    {
            event->mouse_button_callback(button, action, mods);
    }

}

// Dispay frame buffer size callback
void CallbackSetter::frame_buffer_size_callback(GLFWwindow* window, int width,
    int height)
{
    CallbackHandler *event  = 
        reinterpret_cast<CallbackHandler *>(glfwGetWindowUserPointer(window));

    if(event)
    {
            event->frame_buffer_size_callback(width, height);
    }
}