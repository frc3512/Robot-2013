//=============================================================================
//File Name: GeartoothEncoder.cpp
//Description:
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "GeartoothEncoder.hpp"
#include <Timer.h>

GeartoothEncoder::GeartoothEncoder( UINT32 channel , UINT32 teeth , float gearRatio ) : m_counter( channel ) ,
m_rpmAverager( 4 ) ,
m_gearRatio( gearRatio ) ,
m_teeth( teeth ) ,
m_sampleRate( 4.f ) ,
m_isRunning( true ) {
    pthread_mutex_init( &m_dataMutex , NULL );
    pthread_create( &m_sampleThread , NULL , &GeartoothEncoder::threadFunc , this );
}

GeartoothEncoder::~GeartoothEncoder() {
    m_isRunning = false;

    pthread_join( m_sampleThread , NULL );
    pthread_mutex_destroy( &m_dataMutex );
}

void GeartoothEncoder::start() {
    m_counter.Start();
}

void GeartoothEncoder::stop() {
    m_counter.Stop();
}

void GeartoothEncoder::setTeethPerRevolution( UINT32 teeth ) {
    m_teeth = teeth;
}

void GeartoothEncoder::setGearRatio( float ratio ) {
    m_gearRatio = ratio;
}

void GeartoothEncoder::setAverageSize( UINT32 size ) {
    m_rpmAverager.setSize( size );
}

float GeartoothEncoder::getRPM() {
    return m_rpmAverager.getAverage();
}

void GeartoothEncoder::setSampleRate( UINT32 sampleRate ) {
    pthread_mutex_lock( &m_dataMutex );

    m_sampleRate = sampleRate;

    pthread_mutex_unlock( &m_dataMutex );
}

void* GeartoothEncoder::threadFunc( void* object ) {
    GeartoothEncoder* encoderObj = static_cast<GeartoothEncoder*>( object );

    /* Derivation of RPM:
     * period = seconds per tick of gear
     * 1 / period = ticks per second of gear
     * 60 / period = ticks per minute of gear
     * 56 ticks per revolution therefore: 60 / ( 56 * period ) = revolutions per minute of gear
     * 4 * 60 / ( 56 * period ) = revolutions per minute of shooter wheel
     *
     * shooterRPM = 4.f * 60.f / ( 56.f * shooterEncoder.GetPeriod() );
     */

    // Safe copy of sample rate at which to collect data
    float sampleRate = 0.f;

    while ( encoderObj->m_isRunning ) {
        pthread_mutex_lock( &encoderObj->m_dataMutex );

        // Add RPM value to rolling average filter
        encoderObj->m_rpmAverager.addValue( encoderObj->m_gearRatio * 60.f /
                ( encoderObj->m_teeth * encoderObj->m_counter.GetPeriod() ) );

        sampleRate = encoderObj->m_sampleRate;

        pthread_mutex_unlock( &encoderObj->m_dataMutex );

        Wait( 1.f / sampleRate );
    }

    return NULL;
}
