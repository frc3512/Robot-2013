//=============================================================================
//File Name: Shooter.cpp
//Description: Provides an interface to the dual-motored shooter and its
//             encoder; it also has an interface for a PID loop
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "Shooter.hpp"
#include <cmath>
#include <iostream> // TODO Remove me

float Shooter::m_maxSpeed = 10000.f;

Shooter::Shooter( UINT32 motor1 , UINT32 motor2 ,
        UINT32 encChannel , UINT32 encTeeth , float encGearRatio ) :
        m_shooterMotor1( motor1 ) ,
        m_shooterMotor2( motor2 ) ,
        m_shooterEncoder( encChannel , encTeeth , encGearRatio ) ,
        m_shooterPID( 0xFF , 0.f , 0.f , 0.f , this , this ) {
    m_shooterPID.SetOutputRange( 0.f , 1.f );
    m_shooterPID.SetSetpoint( 0.f );
    //m_shooterPID.SetAbsoluteTolerance( 50.f );
    m_shooterEncoder.start();
}

Shooter::~Shooter() {

}

void Shooter::start() {
    m_isShooting = true;
    //m_shooterPID.Enable();
    m_shooterPID.Disable();
    m_shooterEncoder.start();
}

void Shooter::stop() {
    m_isShooting = false;
    m_shooterPID.Disable();
    m_shooterEncoder.stop();

    setRPM( 0 );
}

bool Shooter::isShooting() {
    return m_isShooting;
}

bool Shooter::isReady() {
    return m_shooterPID.OnTarget();
}

void Shooter::setRPM( float wheelSpeed ) {
    m_shooterPID.SetSetpoint( wheelSpeed );
}

float Shooter::getRPM() {
    return m_shooterEncoder.getRPM();
}

void Shooter::setScale( float scaleFactor ) {
    if ( scaleFactor < 0.f || scaleFactor > 1.f ) {
        scaleFactor = fabs( scaleFactor );
    }

    //setRPM( m_maxSpeed * scaleFactor );
    PIDWrite( scaleFactor );
}

float Shooter::getTargetRPM() {
    return m_shooterPID.GetSetpoint();
}

double Shooter::PIDGet() {
    return m_shooterEncoder.getRPM();
}

void Shooter::PIDWrite( float output ) {
    /* Ouputs are negated because the motor controllers require a negative
     * number to make the shooter wheel spin in the correct direction
     */
    m_shooterMotor1.Set( -output );
    m_shooterMotor2.Set( -output );
}
