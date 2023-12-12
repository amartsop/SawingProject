#pragma once
#include <iostream>
#include <filesystem>
#include <memory>
#include <fstream>
#include <nlohmann/json.hpp>

#include <user_interface.h>
#include <joystick_interface.h>
#include <dynamics_math_eigen/dynamics_math_eigen.h>

class NeedleBaseController
{
public:
    NeedleBaseController();
    
    // Set control flag
    void set_control_flag(int control_flag) { m_control_flag = control_flag; }

    // Get needle base pose
    void get_needle_base_pose(Eigen::Vector3d& roa_F_F, Eigen::Vector3d& eul_base)
    {
        roa_F_F = m_roa_F_F; eul_base = m_euler_angles;
    }

    // Set needle base pose 
    void set_needle_base_pose(const Eigen::Vector3d& roa_F_F,
        Eigen::Vector3d& eul_base)
    {
        m_roa_F_F = roa_F_F; m_euler_angles = eul_base;
    }
    
    // Get needle base pose velocity
    void get_needle_base_pose_velocity(Eigen::Vector3d& roa_F_F_dot,
        Eigen::Vector3d& eul_dot_base)
    {
        roa_F_F_dot = m_roa_F_F_dot; eul_dot_base = m_eul_dot;
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

    // Initialize experiment control
    void initialize_experiment_control(const std::string& class_type,
        const std::string& exp_id);

private:

    // Needle base point inertial coordinates
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

    // Control flag (0: Joystick, 1: Gui, 2:...)
    int m_control_flag = 0;

private:

    // Joystick control
    void joystick_control(double delta_time);

    // Base point speed
    double m_js_base_point_speed = 0.01;

    // Euler speed
    double m_js_euler_speed = 0.1;
    
private:

    // Gui control
    void gui_control(double delta_time);

private: 

    // Experiment control
    void experiment_control(double real_time, double delta_time);

    // Experiment translation and rotational velocities
    double m_exp_t_vel, m_exp_r_vel;

    // Experiment duration
    double m_exp_duration;

    // Class type
    std::string m_class_type = "";

    // Experiment type
    std::string m_exp_id = "";

    // Experiment translation and roatational velocities first pass flag
    bool m_exp_first_pass = true;

    // Initial stop time
    double m_init_exp_time= 0.0;

public:

    // Get class type
    std::string get_class_type(void) { return m_class_type; }

    // Get experimet id
    std::string get_experiment_id(void) { return m_exp_id; }

    // Get experimet duration
    double get_experiment_duration(void) { return m_exp_duration; }
};