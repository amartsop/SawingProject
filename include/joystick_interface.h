#pragma once 
#include <iostream>
#include <map>

#include <GLFW/glfw3.h>

// Joystick class for Logitech controller (Switch to D and Mode Off)
class JoystickInterface
{
public:
    JoystickInterface(/* args */);

    // Update
    void update(void);

    // Get joystick values
    std::map<std::string, double> get_joystick_values(void) {
        return m_joy_values;
    }

private:

    // Joystick id (0-15)
    int m_joystick_id;

    // Joystick available flag
    bool m_joy_available_flag;

    // Axes count
    int m_axes_count;

    // Logitech axes count
    const int m_logitech_axes_count = 4;

    // Buttons count
    int m_buttons_count;

    // Logitech buttons count
    const int m_logitech_buttons_count = 16;

private:

    std::map<std::string, double> m_joy_values = 
    {
        {"arrows_horizontal", 0.0},
        {"arrows_vertical", 0.0},
        {"left_joy_horizontal", 0.0},
        {"left_joy_vertical", 0.0},
        {"right_joy_horizontal", 0.0},
        {"right_joy_vertical", 0.0},
        {"button_A", 0.0},
        {"button_B", 0.0},
        {"button_X", 0.0},
        {"button_Y", 0.0},
        {"button_R1", 0.0},
        {"button_R2", 0.0},
        {"button_L1", 0.0},
        {"button_L2", 0.0},
        {"button_START", 0.0},
        {"button_BACK", 0.0},
        {"button_LEFT_JOY", 0.0},
        {"button_RIGHT_JOY", 0.0},
    };

    // Clamp values
    double clamp_values(double x, double limit);
};