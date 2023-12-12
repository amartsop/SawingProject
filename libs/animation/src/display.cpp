#include <iostream>

#include "display.h"

Display::Display(const char* title, int width, int height)
{
    // Member initialization 
    m_width = width; m_height = height;

    // Window state initialization
    m_is_closed = false;

    // GLFW initialization
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window
    m_window = glfwCreateWindow(m_width, m_height, title, NULL, NULL);

    if (m_window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        m_is_closed = true;
        glfwTerminate();
    }

    // Create context 
    glfwMakeContextCurrent(m_window);

    // Initialize glad
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        m_is_closed = true;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // // Setup frame buffer
    // setup_frame_buffer();
}

Display::~Display()
{
    glfwDestroyWindow(m_window);
    glfwTerminate();
}

// Clear display background
void Display::clear(float r, float g, float b, float a)
{
    // Render image 
    glClearColor(r, g, b, a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}


bool Display::is_closed(void)
{
    return m_is_closed;
}

void Display::update(void)
{
    // Swap buffers
    glfwSwapBuffers(m_window);
    glfwMakeContextCurrent(m_window);
    m_is_closed = ((glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS) |
        glfwWindowShouldClose(m_window)); 
}

/********************** Callback functions ***************************/
// Change window size callback
void Display::process_frame_buffer_size(int width, int height)
{
    // Set window width and height
    m_width = width;
    m_height = height;

    glViewport(0, 0, width, height);
}