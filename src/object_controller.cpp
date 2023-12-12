#include "../include/object_controller.h"

ObjectController::ObjectController()
{
    // Initialize base pose
    m_roa_F_F.setZero(); m_euler_angles.setZero();
    m_roa_F_F_dot.setZero(); m_eul_dot.setZero();
}

// Update needle base controller
void ObjectController::update(double real_time, double delta_time)
{
    // Switch between control type
    switch (m_control_flag)
    {
    case 0:
        joystick_control(delta_time);
        break;
    case 1:
        random_control(real_time, delta_time);
        break;
    default:
        joystick_control(delta_time);
        break;
    }
}

// Joystick control
void ObjectController::joystick_control(double delta_time)
{
    // Update joystick values
    m_joystick_values = m_joystick_interface->get_joystick_values();

    // Vector of current position
    Eigen::Vector3d pos = m_roa_F_F;
    
    // Vector of current euler values
    Eigen::Vector3d eul = m_euler_angles;
    
    // Current rotation matrix
    Eigen::Matrix3d rot = dme::EulerRotations::rotation(eul);

    // Update base point
    double dx = m_js_base_point_speed * delta_time *
        (m_joystick_values["button_R1"] - m_joystick_values["button_L1"]);

    double dy = - m_js_base_point_speed * delta_time *
        m_joystick_values["arrows_horizontal"];

    double dz = m_js_base_point_speed * delta_time *
        m_joystick_values["arrows_vertical"];

    Eigen::Vector3d d_pos = {dx, dy, dz};

    // New position
    m_roa_F_F = pos + rot * d_pos;

    // Update euler angles
    double d_phi = m_js_euler_speed * delta_time *
        (m_joystick_values["button_R2"] - m_joystick_values["button_L2"]);

    double d_theta = m_js_euler_speed * delta_time *
        m_joystick_values["right_joy_vertical"];

    double d_psi = - m_js_euler_speed * delta_time *
        m_joystick_values["right_joy_horizontal"];

    Eigen::Vector3d d_eul = {d_phi, d_theta, d_psi};

    // New orientation        
    m_euler_angles = eul + d_eul;
}

// Random control
void ObjectController::random_control(double real_time, double delta_time)
{
    // Check if we should update
    if (real_time - m_random_previous_time >= m_random_update_interval)
    {
        // Generate random x, y and z values
        m_roa_F_F(0) = Utils::random_double_in_range(0.4, 0.44);
        m_roa_F_F(1) = Utils::random_double_in_range(0.1, 0.8);
        m_roa_F_F(2) = Utils::random_double_in_range(0.05, 0.12);

        // Generate random euler angles
        m_euler_angles(0) = Utils::random_double_in_range(0.0, 2.0 * M_PI);
        m_euler_angles(1) = Utils::random_double_in_range(0.0, 2.0 * M_PI);
        m_euler_angles(2) = Utils::random_double_in_range(0.0, 2.0 * M_PI);

    
        // Update previous time
        m_random_previous_time = real_time;
    }
}