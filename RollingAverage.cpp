//=============================================================================
//File Name: RollingAverage.cpp
//Description: Creates queue of values and returns the average of the latest
//             values added to it
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "RollingAverage.hpp"

RollingAverage::RollingAverage( unsigned int size ) :
m_size( size ) {

}

RollingAverage::~RollingAverage() {

}

void RollingAverage::addValue( float value ) {
    m_values.push_back( value );

    while ( m_values.size() > m_size ) {
        m_values.pop_front();
    }
}

void RollingAverage::setSize( unsigned int size ) {
    // While the new size is less than old size, remove values
    while ( size < m_size ) {
        m_values.pop_front();
    }

    m_size = size;
}

float RollingAverage::getAverage() {
    float count = 0;

    std::list<float>::iterator i;
    for ( i = m_values.begin() ; i != m_values.end() ; i++ ) {
        count += *i;
    }

    return count / m_values.size();
}
