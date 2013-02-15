//=============================================================================
//File Name: RollingAverage.cpp
//Description: Creates queue of values and returns the average of the latest
//             values added to it
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "RollingAverage.hpp"

RollingAverage::RollingAverage( unsigned int size ) :
m_size( size ) {
    pthread_mutex_init( &m_dataMutex , NULL );
}

RollingAverage::~RollingAverage() {
    pthread_mutex_destroy( &m_dataMutex );
}

void RollingAverage::addValue( float value ) {
    pthread_mutex_lock( &m_dataMutex );

    m_values.push_back( value );

    while ( m_values.size() > m_size ) {
        m_values.pop_front();
    }

    pthread_mutex_unlock( &m_dataMutex );
}

void RollingAverage::setSize( unsigned int size ) {
    pthread_mutex_lock( &m_dataMutex );

    // While the new size is less than old size, remove values
    while ( size < m_size ) {
        m_values.pop_front();
    }

    m_size = size;

    pthread_mutex_unlock( &m_dataMutex );
}

float RollingAverage::getAverage() {
    float count = 0;

    pthread_mutex_lock( &m_dataMutex );

    std::list<float>::iterator i;
    for ( i = m_values.begin() ; i != m_values.end() ; i++ ) {
        count += *i;
    }

    float returnVal = count / m_values.size();

    pthread_mutex_unlock( &m_dataMutex );

    return returnVal;
}
