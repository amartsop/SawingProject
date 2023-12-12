#include "../include/application_interface.h"

// Initialize application
ApplicationInterface::ApplicationInterface(void)
{
    // Initialize display 
    m_display = std::make_shared<Display>(m_window_title.c_str());

    // World interface
    m_world_interface = std::make_shared<WorldInterface>(m_sim_update_interval);

    // Set world interface to animation interface
    m_anim_interface.set_world_interface(m_world_interface);

    // Initialize animation 
    m_animation.initialize(&m_anim_interface, m_display);

    // Initialzie user interface
    m_gui.initialize(m_display);

    // Joystick interface
    m_joystick_interface = std::make_shared<JoystickInterface>();

    // Set user interface
    m_world_interface->initialize_gui(&m_gui);

    // Set joystick
    m_world_interface->initialize_joystick(m_joystick_interface);

    // // Initialize video recorder
    // m_video_recorder.initialize("./video.mp4", m_display->get_window_width(), 
    //     m_display->get_window_height(), 10);
}

// Update application
void ApplicationInterface::update(double real_time, double delta_time)
{
    // Update simulation
    update_simulation(real_time, delta_time);

    // Update animation
    update_animation(real_time, delta_time);
}

// Update simulation
void ApplicationInterface::update_simulation(double real_time, double delta_time)
{
    // Check if we should update simulation
    if (real_time - m_sim_update_interval)
    {
        // Update world interface
        m_world_interface->update(real_time, delta_time);

        // Update previous time
        m_sim_update_interval = real_time;
    }
}

// Update animation
void ApplicationInterface::update_animation(double real_time, double delta_time)
{
    // Check if we should render animations
    if (real_time - m_prev_anim_time)
    {
        // Update animation
        m_animation.update(real_time, delta_time);

        // // Update gui
        // m_gui.update();

        // Update joystick
        m_joystick_interface->update();

        // // Update video recorder
        // m_video_recorder.update(real_time);

        // Update display
        m_display->update();

        // Update previous time
        m_prev_anim_time = real_time;
    }
}

