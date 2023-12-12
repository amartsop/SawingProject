#pragma once
#include <iostream>
#include <filesystem>
#include <memory>
#include <fstream>
#include <nlohmann/json.hpp>

#include <utils.h>
#include <user_interface.h>
#include <joystick_interface.h>
#include <dynamics_math_eigen/dynamics_math_eigen.h>

class ObjectController
{
public:
    ObjectController();
    
    // Set control flag
    void set_control_flag(int control_flag) { m_control_flag = control_flag; }

    // Get object pose
    void get_object_pose(Eigen::Vector3d& roa_F_F, Eigen::Vector3d& eul_base)
    {
        roa_F_F = m_roa_F_F; eul_base = m_euler_angles;
    }
    
    // Update needle base controller
    void update(double real_time, double delta_time);
    
    // Initialize joystick
    inline void initialize_joystick(const std::shared_ptr<JoystickInterface>&
        joystick_interface)
    {
        m_joystick_interface = joystick_interface;
    }

    // Initialize gui
    inline void initialize_gui(UserInterface* gui) { m_gui = gui; }


private:

    // Object inertial coordinates
    Eigen::Vector3d m_roa_F_F;

    // Euler angles
    Eigen::Vector3d m_euler_angles;

    // Needle base point velocity
    Eigen::Vector3d m_roa_F_F_dot;

    // Needle base euler angles velocity
    Eigen::Vector3d m_eul_dot;

private:

    // Joystick interface
    std::shared_ptr<JoystickInterface> m_joystick_interface; 

    // Joystick values
   std::map<std::string, double> m_joystick_values;
    
    // User interface pointer
    UserInterface* m_gui;

    // Control flag (0: Joystick, 1: Random, 2:...)
    int m_control_flag = 0;

private:

    // Joystick control
    void joystick_control(double delta_time);

    // Base point speed
    double m_js_base_point_speed = 0.1;

    // Euler speed
    double m_js_euler_speed = 0.1;
    
private:

    // Random control
    void random_control(double real_time, double delta_time);

    // Random update interval (s)
    double m_random_update_interval = 0.5;

    // Previous update time
    double m_random_previous_time = 0.0;
};