#pragma once 

#include <iostream>
#include <filesystem>
#include <map>
#include <chrono>

#include <eigen3/Eigen/Dense>
#include <igl/readOBJ.h>

#include <user_interface.h>
#include <joystick_interface.h>
#include <object_controller.h>
#include <data_generator.h>

class WorldInterface
{
public:
    WorldInterface(double ts);

    // Set user interface
    void initialize_gui(UserInterface* gui);

    // Update
    void update(double real_time, double delta_time);

    // Initialize joystick
    void initialize_joystick(const std::shared_ptr<JoystickInterface>&
        joystick_interface);

    // Get object pose
    void get_object_pose(Eigen::Vector3d& roa_F_F, Eigen::Vector3d& eul_base)
    {
        m_object_controller->get_object_pose(roa_F_F, eul_base);
    }

private:
    
    // Current filepath
    std::filesystem::path m_current_path;    

private:

    // User interface pointer
    UserInterface* m_gui;

    // Joystick interface
    std::shared_ptr<JoystickInterface> m_joystick_interface; 

   // Needle base controller
    std::shared_ptr<ObjectController> m_object_controller;

    // Joystick values
    std::map<std::string, double> m_joystick_values;

    // Data generator
    std::shared_ptr<DataGenerator> m_data_generator;
};