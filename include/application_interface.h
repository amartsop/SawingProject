#pragma once

#include <iostream>
#include <memory>
#include <filesystem.h>

#include <display.h>
#include <scene.hpp>
#include <animation_interface.h>
#include <user_interface.h>
#include <world_interface.h>
#include <joystick_interface.h>
// #include <video_recorder.h>


class ApplicationInterface
{
public:
    ApplicationInterface();

    // Update application
    void update(double real_time, double deltaTime);

    // Check if animation is done
    bool is_done(void) {
        return (m_display->is_closed());
    }

private:
    
    // Animation interface
    AnimationInterface m_anim_interface;

    // Define animation
    Scene<AnimationInterface> m_animation;

    // Window properties
    const std::string m_window_title = "Sawing Environment";
    const float m_background_color[3] = {0.82f, 0.8f, 0.9f}; // RGB
        
    // Display object ptr
    std::shared_ptr<Display> m_display;

private:
    
    /// User interface
    UserInterface m_gui;

    // World interface
    std::shared_ptr<WorldInterface> m_world_interface;

    // Joystick interface
    std::shared_ptr<JoystickInterface> m_joystick_interface;

    // // Video recorder
    // VideoRecorder m_video_recorder;

private:

    // Update simulation
    void update_animation(double real_time, double delta_time);

    // Animation previous time (sec)
    double m_prev_anim_time = 0.0;

    // Animation update interval (sec)
    double m_anim_update_interval = 1.0 / 30.0;

private:

    // Update simulation
    void update_simulation(double real_time, double delta_time);

    // Simmulation previous time (sec)
    double m_prev_sim_time = 0.0;

    // Simmulation previous time (sec)
    double m_sim_update_interval = 1.0 / 100.;
};