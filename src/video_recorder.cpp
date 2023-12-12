#include "../include/video_recorder.h"

void VideoRecorder::initialize(std::string filename, int width, int height, int fps)
{
    // Set video properties
    m_filename = filename; m_width = width; m_height = height; m_fps = fps;

    // Set update interval
    m_update_interval = 1.0 / (double)m_fps;
}

void VideoRecorder::update(double real_time)
{
    // Check for pending recording status
    if (m_recording_pending_start)
    {
        /******************* Command to ffmpeg *******************/
        // FPS command 
        std::string fps_cmd = "-r " + std::to_string(m_fps);

        // Resolution command 
        std::string res_cmd = "-s " + std::to_string(m_width) + "x" +
            std::to_string(m_height);

        // Final command (string format)
        m_cmd = m_command + m_space + m_verb_opt + m_space + fps_cmd
            + m_space + m_force_opt + m_space + m_color + m_space + res_cmd +
            m_space + m_rest + m_space + m_filename;

        // Send commands to bash
        m_ffmpeg = popen(m_cmd.c_str(), "w");

        // Allocate memory for pixel data
        m_buffer = new int[m_width * m_height];

        // Set init time 
        m_init_time = real_time;

        // Deactivate pending start
        m_recording_pending_start = false;
    }
    
    //Record video
    glReadPixels(0, 0, m_width, m_height, GL_RGBA, GL_UNSIGNED_BYTE, m_buffer);
    fwrite(m_buffer, sizeof(int) * m_width * m_height, 1, m_ffmpeg);
    
    // Update previous time
    m_previous_update_time = real_time;

    // if (real_time - m_previous_update_time >= m_update_interval)
    // {
    //     //Record video
    //     glReadPixels(0, 0, m_width, m_height, GL_RGBA, GL_UNSIGNED_BYTE, m_buffer);
    //     fwrite(m_buffer, sizeof(int) * m_width * m_height, 1, m_ffmpeg);
    
    //     // Update previous time
    //     m_previous_update_time = real_time;
    // } 
}

VideoRecorder::~VideoRecorder()
{
    if (!m_stop_execution)
    {
        delete[] m_buffer;
        pclose(m_ffmpeg);
    }
}
