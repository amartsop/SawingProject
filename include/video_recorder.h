#pragma once 

#include <iostream>
#include <stdio.h>
#include <glad/glad.h>

class VideoRecorder
{
public:

    VideoRecorder() {};

    // Initialize
    void initialize(std::string filename, int width, int height, int fps=30);

    // Update video recorder
    void update(double real_time);
    
    ~VideoRecorder();

private:

    // Command
    const std::string m_command = "ffmpeg";

    // Verbose option
    const std::string m_verb_opt = "-v 0";

    // Force format option
    const std::string m_force_opt = "-f rawvideo";

    // Color option
    const std::string m_color = "-pix_fmt rgba";

    // Rest of commands and options
    const std::string m_rest = "-i - -threads 0 -preset fast -y -pix_fmt "
        "yuv420p -crf 21 -vf vflip";

    // Space 
    const std::string m_space = " ";
    
private:

    // Execution command
    std::string m_cmd;

    // Recording pending start
    bool m_recording_pending_start = true;

    // Stop execution flag
    bool m_stop_execution = false;

private:
    // Image buffer pointer
    int* m_buffer;

    // File pointer
    FILE* m_ffmpeg;

private:

    // Video propeties
    int m_width, m_height, m_fps;

    // Update interval (sec)
    double m_update_interval = 0.0; 
    
    // Video filename
    std::string m_filename;

    // Previous update time
    double m_previous_update_time = 0.0;

    // Init time 
    double m_init_time = 0.0;

};