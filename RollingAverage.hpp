//=============================================================================
//File Name: RollingAverage.hpp
//Description: Creates queue of values and returns the average of the latest
//             values added to it
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef ROLLING_AVERAGE_HPP
#define ROLLING_AVERAGE_HPP

#include <pthread.h>
#include <atomic>

#include "RWProtect.hpp"

class RollingAverage {
public:
    RollingAverage( unsigned int size );
    virtual ~RollingAverage();

    void addValue( float value );
    void setSize( unsigned int newSize );
    float getAverage();

private:
    // Holds values to be averaged
    //std::list<float> m_values;
    std::atomic<std::atomic<float>*> m_values;

    // Determines oldest value in array
    std::atomic<unsigned int> m_index;

    // Number of values to average
    std::atomic<unsigned int> m_size;

    // Total number of slots available in array (may be larger than m_size)
    std::atomic<unsigned int> m_maxSize;

    RWProtect m_protectArray;

    // Prevents accessing list from two places at once
    //std::mutex m_dataMutex;
    pthread_mutex_t m_dataMutex;
};

#endif // ROLLING_AVERAGE_HPP
