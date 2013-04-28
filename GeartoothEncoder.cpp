//=============================================================================
//File Name: GeartoothEncoder.cpp
//Description: Counts the number of gear teeth which have passed using a Hall's
//             Effect sensor plugged into a DIO channel and returns the RPM.
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "GeartoothEncoder.hpp"

GeartoothEncoder::GeartoothEncoder( UINT32 channel , UINT32 teeth , float gearRatio ) : m_counter( channel ) ,
m_rpmAverager( 2 ) ,
m_gearRatio( gearRatio ) ,
m_teeth( teeth ) {
    m_sampleThread = new Notifier( &GeartoothEncoder::threadFunc , this );
    m_sampleThread->StartPeriodic( 0.25f );
}

GeartoothEncoder::~GeartoothEncoder() {
    delete m_sampleThread;
}

void GeartoothEncoder::start() {
    m_counter.Start();
}

void GeartoothEncoder::stop() {
    m_counter.Stop();
}

void GeartoothEncoder::setTeethPerRevolution( UINT32 teeth ) {
    m_dataMutex.take();

    m_teeth = teeth;

    m_dataMutex.give();
}

void GeartoothEncoder::setGearRatio( float ratio ) {
    m_dataMutex.take();

    m_gearRatio = ratio;

    m_dataMutex.give();
}

void GeartoothEncoder::setAverageSize( UINT32 size ) {
    m_rpmAverager.setSize( size );
}

float GeartoothEncoder::getRPM() {
    return m_rpmAverager.getAverage();
}

void GeartoothEncoder::setSampleRate( UINT32 sampleRate ) {
    m_sampleThread->StartPeriodic( 1.f / sampleRate );
}

void GeartoothEncoder::threadFunc( void* object ) {
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

    encoderObj->m_dataMutex.take();

    // Add RPM value to rolling average filter
    encoderObj->m_rpmAverager.addValue( encoderObj->m_gearRatio * 60.f /
            ( encoderObj->m_teeth * encoderObj->m_counter.GetPeriod() ) );

    encoderObj->m_dataMutex.give();
}
