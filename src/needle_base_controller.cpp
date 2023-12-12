#include "../include/needle_base_controller.h"

NeedleBaseController::NeedleBaseController()
{
    // Initialize base pose
    m_roa_F_F.setZero(); m_euler_angles.setZero();
    m_roa_F_F_dot.setZero(); m_eul_dot.setZero();
}

// Update needle base controller
void NeedleBaseController::update(double real_time, double delta_time)
{
    // Switch between control type
    switch (m_control_flag)
    {
    case 0:
        joystick_control(delta_time);
        break;
    case 1:
        gui_control(delta_time);
        break;
    case 2:
        experiment_control(real_time, delta_time);
        break;
    default:
        joystick_control(delta_time);
        break;
    }
}

// Initialize experiment control
void NeedleBaseController::initialize_experiment_control(
    const std::string& class_type, const std::string& exp_id)
{
    // Set class type     
    m_class_type = class_type;

    // Set experiment id 
    m_exp_id = exp_id;

    // Current json file 
    std::filesystem::path current_path = std::filesystem::current_path();

    // Relative json file 
    std::filesystem::path rel_path = "share/experiments_info.json";

    // Absolute json filename
    std::string abs_json_path = (current_path / rel_path).string();

    // Read experiments info file
    std::ifstream f(abs_json_path);
    nlohmann::json json_data = nlohmann::json::parse(f);

    // Read translational insertion velocity
    m_exp_t_vel = json_data[class_type][exp_id]["t_vel"];

    // Read rotational insertion velocity
    m_exp_r_vel = json_data[class_type][exp_id]["r_vel"];
    
    // Read duration
    m_exp_duration = json_data[class_type][exp_id]["duration"];
}

// Experiment control
void NeedleBaseController::experiment_control(double real_time, double delta_time)
{
    // Vector of current position
    Eigen::Vector3d pos = m_roa_F_F;
    
    // Vector of current euler values
    Eigen::Vector3d eul = m_euler_angles;
    
    // Rotation matrix
    Eigen::Matrix3d rot_f_F = dme::EulerRotations::rotation(eul);

    // Set values to zero if surpass experiment duration
    if (real_time >= m_exp_duration) {

        if (m_exp_first_pass)
        {
            // Define first pass flag
            m_init_exp_time = real_time;
        
            // Update first pass flag
            m_exp_first_pass = false;
        }

        // Execution time
        double exec_time = real_time - m_init_exp_time;

        m_exp_t_vel *= (1 - tanh(0.1 * exec_time));
        m_exp_r_vel *= (1 - tanh(0.1 * exec_time));
    }

    /******************** Translational Velocity ********************/ 
    m_roa_F_F_dot = {m_exp_t_vel, 0.0, 0.0};
    
    /******************** Translational Positions ********************/ 
    // Position x, y and z (m)
    m_roa_F_F = m_roa_F_F + delta_time * m_roa_F_F_dot;
    
    /******************** Roatational Velocities ********************/ 
    // Rotational velocity
    Eigen::Vector3d w_f_F_f = {m_exp_r_vel, 0.0, 0.0};

    // Matrix G
    Eigen::Matrix3d g_mat = dme::EulerRotations::G(eul);

    // Euler rate of change
    m_eul_dot = g_mat.inverse() * w_f_F_f;
    
    /******************** Euler Angles ********************/ 
    // Euler angle around x, y and z (rad)
    m_euler_angles = m_euler_angles + delta_time * m_eul_dot;
}

// Joystick control
void NeedleBaseController::joystick_control(double delta_time)
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

    // Orientation perturbation
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

// Gui control
void NeedleBaseController::gui_control(double delta_time)
{
    // Get euler angles
    m_euler_angles = m_gui->get_needle_base_orienation();

    // Adgust local position
    m_roa_F_F = dme::EulerRotations::rotation(m_euler_angles) *
        m_gui->get_needle_base_position();
}