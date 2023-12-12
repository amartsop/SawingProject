#pragma once 

#include <iostream>
#include <fstream>

#include <vector>
#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <filesystem.h>
#include <display.h>
#include <callback_setter.h>
#include <callback_handler.h>
#include <mesh.h>
#include <shader.h>
#include <transform.h>
#include <texture.h>


template <class T>
class Scene
{
public:
    Scene(){};

    // Initialize animation
    void initialize(T* object_handler, std::shared_ptr<Display> display);

    // Update scene
    void update(double real_time, double deltaTime);

    // Get simple shader path
    std::string get_simple_shader_path(void) { return m_scene_shader_name; }

private:

    // Display pointer
    std::shared_ptr<Display> m_display;

    // Filepath
    std::string absolute_path = FileSystem::get_absolute_path();
        
    // Window handle
    GLFWwindow* m_window; 

    // Camera constants
    const glm::vec3 m_camera_pos = glm::vec3(-0.2f, 0.4f, 0.3f);
    const float m_camera_fov = 45.0f; const float m_z_near = 0.01f;
    const float m_z_far = 1000.0f;

    // Camera object
    std::shared_ptr<Camera> m_camera;

    // Callback functions
    std::shared_ptr<CallbackSetter> m_callback_set;
    std::shared_ptr<CallbackHandler> m_callback_handler;

    // Shader constants
    const std::string m_scene_shader_name = absolute_path + "/share/simpleShader";

    // Shader object
    Shader m_scene_shader;

    // Initial light position
    glm::vec3 m_light_pos = glm::vec3(0.1f, 0.0f, 0.4f);

private:

    // Update lighting and camera setup
    void update_setup(void);

    // Object handler
    T* m_object_handler;

private:

    // Window properties
    const char* m_window_title = "Transperineal Prostate Biopsy";
    const float m_background_color[3] = {0.82f, 0.8f, 0.9f}; // RGB
};


template <class T>
void Scene<T>::initialize(T* object_handler, std::shared_ptr<Display> display)
{
    // Get display pointer
    m_display = display; 

    // Get window handle
    m_window = m_display->get_window_handle();

    // Initialize camera
    m_camera = std::make_shared<Camera>(m_window, m_camera_pos, m_camera_fov,
        (float)m_display->get_window_width() / (float)m_display->get_window_height(),
        m_z_near, m_z_far);

    // Initialize callbacks
    m_callback_set = std::make_shared<CallbackSetter>(m_window);
    m_callback_handler = std::make_shared<CallbackHandler>(m_window);

    // Set cursor callback
    m_callback_handler->set_cursor_coords(
        (float)m_display->get_window_width() / 2.0f, 
        (float)m_display->get_window_height() / 2.0f);

    // Set display callback inputs
    m_callback_handler->set_display(m_display.get());

    // Set camera callback inputs
    m_callback_handler->set_camera(m_camera);

    // Initialize shaders
    m_scene_shader.initialize(m_scene_shader_name);

    // Copy object handler
    m_object_handler = object_handler;

    // Initialize animation interface
    m_object_handler->initialize_animation(this);
}

template <class T>
void Scene<T>::update(double real_time, double delta_time)
{
    // Poll events
    glfwPollEvents();

    /**************** Display ****************/
    // Clear display background
    m_display->clear(m_background_color[0], m_background_color[1],
        m_background_color[2]);

    /**************** Camera ****************/
    // Update camera 
    m_camera->update(delta_time);

    // Bind shader 
    m_scene_shader.bind();

    // Update setup
    update_setup();

    // Update geometries
    m_object_handler->update(real_time);
}

// Update lighting and camera setup
template <class T>
void Scene<T>::update_setup(void)
{
    // Camera and light position
    m_scene_shader.set_vec3("viewPos", m_camera->get_camera_position());
    m_scene_shader.set_vec3("light.position", m_light_pos);

    // Light properties
    m_scene_shader.set_vec3("light.ambient", 0.5f, 0.5f, 0.5f); 
    m_scene_shader.set_vec3("light.diffuse", 0.2f, 0.2f, 0.2f);
    m_scene_shader.set_vec3("light.specular", 0.2f, 0.2f, 0.2f);

    // View and projection
    m_scene_shader.set_mat4("projection", m_camera->get_projection());
    m_scene_shader.set_mat4("view", m_camera->get_view());
}