#pragma once

#include <iostream>
#include <vector>
#include <memory>

#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

#include "display.h"
#include <eigen3/Eigen/Dense>

class UserInterface
{
public:
    UserInterface(void){};
        
    /// Initialize 
    void initialize(std::shared_ptr<Display> display);
        
    // Update gui
    void update(void);

    // Destructor
    ~UserInterface();

public:

    void simple_window(void);

public:

    // // Set sphere initial position
    // void set_point_initial_position(const Eigen::Vector3d& pos0)
    // {
    //     m_sphere_pos[0] = pos0(0); m_sphere_pos[1] = pos0(1);
    //     m_sphere_pos[2] = pos0(2);
    // }
    
    // Get point position
    Eigen::Vector3d get_needle_base_position(void);

    // Get point orienation
    Eigen::Vector3d get_needle_base_orienation(void);

private:
    
    // Position step
    float m_position_step = 0.001f;

    // Euler step
    float m_euler_step = 0.01;

private:    

    // Needle position
    float m_needle_pos0[3] = {0.0f, 0.0f, 0.0f};   

    // Needle position
    float m_needle_pos[3] = {0.0f, 0.0f, 0.0f};

    // Needle translation names
    std::vector<std::string> m_needle_translation_names = {"x (m)",
        "y (m)", "z (m)"};

    // Initial needle orintation
    float m_needle_eul0[3] = {0.0f, 0.0f, 0.0f};   

    // Needle euler
    float m_needle_euler[3] = {0.0f, 0.0f, 0.0f};

    // Needle rotation names
    std::vector<std::string> m_needle_rotation_names = {"Phi (rad)",
        "Theta (rad)", "Psi (rad)"};

private:
        
    // Display pointer
    std::shared_ptr<Display> m_display;

    // Display width and height
    int m_display_width, m_display_height;
    
    // Set ImGui Style
    void set_ImGui_style(void);
};