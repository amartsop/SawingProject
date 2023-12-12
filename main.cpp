#include <iostream>

#include <cstdlib>
#include <iostream>
#include <armadillo>
#include <ostream>
#include <vector>
#include <chrono>
#include <thread>

#include "./include/application_interface.h"


int main(int argc, char** argv)
{
    // Initialize application interface
    ApplicationInterface app;

    // Clock
    auto previous_time = std::chrono::steady_clock::now();
    double real_time = 0.0;

    /************************* Main loop **************************/
    while(!app.is_done())
    {
        // Current time
        auto current_time = std::chrono::steady_clock::now();   

        // Elapsed time
        std::chrono::duration<double> elapsed_time = current_time - previous_time;

        // Real time 
        real_time += elapsed_time.count();
        
        // Render application 
        app.update(real_time, elapsed_time.count());
        
        // Update time 
        previous_time = current_time;
    }
}