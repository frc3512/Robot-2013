//=============================================================================
//File Name: RollingAverage.hpp
//Description: Creates queue of values and returns the average of the latest
//             values added to it
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef ROLLING_AVERAGE_HPP
#define ROLLING_AVERAGE_HPP

#include <list>

class RollingAverage {
public:
    RollingAverage( unsigned int size );
    virtual ~RollingAverage();

    void addValue( float value );
    void setSize( unsigned int size );
    float getAverage();

private:
    // Holds values to be averaged
    std::list<float> m_values;

    // Number of values to average
    unsigned int m_size;
};

#endif // ROLLING_AVERAGE_HPP
