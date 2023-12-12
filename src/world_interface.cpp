#include "../include/world_interface.h"

WorldInterface::WorldInterface(double ts)
{
    // Get current filepath
    m_current_path = std::filesystem::current_path();
}

// Update
void WorldInterface::update(double real_time, double delta_time)
{
}

// Set user interface
void WorldInterface::initialize_gui(UserInterface* gui)
{
    // Set gui handle
    m_gui = gui;
}

// Seyt joystick joystick
void WorldInterface::initialize_joystick(const
    std::shared_ptr<JoystickInterface>& joystick_interface)
{
    // Store joystick interface
    m_joystick_interface = joystick_interface;
}