#include "../include/joystick_interface.h"

JoystickInterface::JoystickInterface(/* args */)
{
    // Loop through all potential joysticks (16 in total for glfw)
    for (int i = 0; i < 16; i++)
    {
        int joy_flag = glfwJoystickPresent(i);

        if (joy_flag == GLFW_TRUE)
        {
            // Get joystick id
            m_joystick_id = i;

            // Update flag
            m_joy_available_flag = true;
        }
    }

    // Log that no joystick was found
    if (!m_joy_available_flag)
    {
        std::cout << "No joystick was found!" << std::endl;
    }
}

// Update joystick
void JoystickInterface::update(void)
{
    // Get joint axes values
    const float* axes_values = glfwGetJoystickAxes(m_joystick_id, &m_axes_count);

    if (m_axes_count == m_logitech_axes_count)
    {
        // Left joystick horizontal
        m_joy_values["left_joy_horizontal"] = clamp_values(axes_values[0], 0.01);
        
        // // Left joystick vertical
        m_joy_values["left_joy_vertical"] = clamp_values(axes_values[1], 0.01);

        // Right joystick vertical
        m_joy_values["right_joy_horizontal"] = clamp_values(axes_values[2], 0.01);
    
        // Right joystick vertical
        m_joy_values["right_joy_vertical"] = clamp_values(axes_values[3], 0.01);
    }
    
    
    
    // Get button vaues
    const unsigned char* button_values = glfwGetJoystickButtons(m_joystick_id,
        &m_buttons_count);
    
    // Set buttons
    if (m_buttons_count == m_logitech_buttons_count)
    {
        // Button A
        m_joy_values["button_A"] = (button_values[1] == GLFW_PRESS);

        // Button X
        m_joy_values["button_X"] = (button_values[0] == GLFW_PRESS);

        // Button Y
        m_joy_values["button_Y"] = (button_values[3] == GLFW_PRESS);

        // Button B
        m_joy_values["button_B"] = (button_values[2] == GLFW_PRESS);

        // Arrows horizontal
        m_joy_values["arrows_horizontal"] = (button_values[13] == GLFW_PRESS) -
            (button_values[15] == GLFW_PRESS);
    
        // Arrows vertical
        m_joy_values["arrows_vertical"] = (button_values[12] == GLFW_PRESS) -
            (button_values[14] == GLFW_PRESS);
    
        // Button R1
        m_joy_values["button_R1"] = (button_values[5] == GLFW_PRESS);

        // Button R2
        m_joy_values["button_R2"] = (button_values[7] == GLFW_PRESS);

        // Button L1
        m_joy_values["button_L1"] = (button_values[4] == GLFW_PRESS);

        // Button L2
        m_joy_values["button_L2"] = (button_values[6] == GLFW_PRESS);
    
        // Button right joy
        m_joy_values["button_RIGHT_JOY"] = (button_values[11] == GLFW_PRESS);

        // Button left joy
        m_joy_values["button_LEFT_JOY"] = (button_values[10] == GLFW_PRESS);

        // Button start
        m_joy_values["button_START"] = (button_values[9] == GLFW_PRESS);

        // Button back
        m_joy_values["button_BACK"] = (button_values[8] == GLFW_PRESS);
    }
}

double JoystickInterface::clamp_values(double x, double limit) 
{
    if (std::abs(x) <= std::abs(limit))
    {
        x = 0.0; return x;
    }
    else
    {
        return x;
    }
}