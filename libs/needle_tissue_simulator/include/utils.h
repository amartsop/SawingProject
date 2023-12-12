#pragma once
#include<vector>
#include <algorithm>
#include <numeric>


namespace utils
{
    // System state
    enum SystemState { outside, stiffness, insertion, retraction };

    // Sign function
    double sign_func(double x);
    
    // Sliding window
    class SlidingWindow
    {
        public:
            SlidingWindow(int window_size)
            {
                // Store window size
                m_window_size = window_size;
            }
    
            // Push function
            void push(double data)
            {
                if (m_window.size() < m_window_size)
                {
                    m_window.push_back(data);
                }
                else
                {
                    m_window.erase(m_window.begin());
                    m_window.push_back(data);
                }

                double sum = std::accumulate(std::begin(m_window),
                    std::end(m_window), 0.0);
                m_window_mean = sum / m_window.size();
            }
    
            // Get mean
            double get_mean(void) { return m_window_mean; }
    
        private:

            // Sliding window size
            int m_window_size;
    
            // Sliding window
            std::vector<double> m_window;

            // Calculate window mean
            double m_window_mean;
    };
} 
