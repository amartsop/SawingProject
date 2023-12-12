#include "../include/world_interface.h"

WorldInterface::WorldInterface(double ts)
{
    // Get current filepath
    m_current_path = std::filesystem::current_path();

    // Control type
    int control_type = 1;
    
    // Object controller
    m_object_controller = std::make_shared<ObjectController>();

    // Set controller type
    m_object_controller->set_control_flag(control_type);

    // Data generator
    m_data_generator = std::make_shared<DataGenerator>();
}

// Update
void WorldInterface::update(double real_time, double delta_time)
{
    // Update object controller
    m_object_controller->update(real_time, delta_time);
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

   // Pass joystick handle to needle base controller
    m_object_controller->initialize_joystick(joystick_interface);
}