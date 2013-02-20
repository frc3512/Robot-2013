//=============================================================================
//File Name: Shooter.cpp
//Description: Provides an interface to the dual-motored shooter and its
//             encoder; it also has an interface for a PID loop
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "Shooter.hpp"
#include <cmath>
#include <cfloat>

const float Shooter::maxSpeed = 5000.f;

Shooter::Shooter( UINT32 motor1 , UINT32 motor2 ,
        UINT32 encChannel , UINT32 encTeeth , float encGearRatio ) :
        m_shooterMotor1( motor1 ) ,
        m_shooterMotor2( motor2 ) ,
        m_shooterEncoder( encChannel , encTeeth , encGearRatio ) ,
        m_shooterPID( 0.001f , 0.f , 0.f , 0.f , this , this ) , // 0.01f , 0.1f , 0.005f
        m_setpoint( 0.f ) {
    m_shooterPID.SetOutputRange( 0.f , 100.f );
    m_shooterPID.SetSetpoint( 0.f );
    m_shooterPID.SetTolerance( 0.f );
    m_shooterEncoder.start();

    m_controlMode = PID;
}

Shooter::~Shooter() {

}

void Shooter::start() {
    m_isShooting = true;
    enableControl();
    m_shooterEncoder.start();
}

void Shooter::stop() {
    m_isShooting = false;
    disableControl();
    m_shooterEncoder.stop();

    setRPM( 0 );
}

bool Shooter::isShooting() {
    return m_isShooting;
}

bool Shooter::isReady() {
    return false;
}

void Shooter::setRPM( float wheelSpeed ) {
    m_setpoint = wheelSpeed;
    m_shooterPID.SetSetpoint( wheelSpeed );
}

float Shooter::getRPM() {
    return m_shooterEncoder.getRPM();
}

void Shooter::setScale( float scaleFactor ) {
    if ( m_isShooting ) {
        if ( scaleFactor < 0.f || scaleFactor > 1.f ) {
            scaleFactor = fabs( scaleFactor );
        }

        PIDWrite( scaleFactor );
    }
}

float Shooter::getTargetRPM() {
    return m_setpoint;
}

void Shooter::enableControl() {
    //if ( getControlMode() == PID ) {
        m_shooterPID.Enable();
    //}
}

void Shooter::disableControl() {
    //if ( getControlMode() == PID ) {
        m_shooterPID.Disable();
    //}
}

void Shooter::setControlMode( ControlMode mode ) {
    m_controlMode = mode;
}

Shooter::ControlMode Shooter::getControlMode() {
    return m_controlMode;
}

double Shooter::PIDGet() {
    return m_shooterEncoder.getRPM();
}

void Shooter::PIDWrite( float output ) {
    /* Ouputs are negated because the motor controllers require a negative
     * number to make the shooter wheel spin in the correct direction
     */
    switch ( m_controlMode ) {
    case PID: {
        m_shooterMotor1.Set( -output );
        m_shooterMotor2.Set( -output );

        break;
    }

    case BangBang: {
        if ( m_shooterEncoder.getRPM() >= m_setpoint ) {
            m_shooterMotor1.Set( 0.f );
            m_shooterMotor2.Set( 0.f );
        }
        else {
            m_shooterMotor1.Set( -1.f );
            m_shooterMotor2.Set( -1.f );
        }

        break;
    }
    }
}
