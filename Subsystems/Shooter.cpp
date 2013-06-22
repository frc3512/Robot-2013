//=============================================================================
//File Name: Shooter.cpp
//Description: Provides an interface to the dual-motored shooter and its
//             encoder; it also has an interface for a PID loop
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "Shooter.hpp"
#include <cmath>
#include <cfloat>

#include "../SFML/System/Lock.hpp"

const float Shooter::maxSpeed = 5000.f;

Shooter::Shooter( UINT32 motor1 , UINT32 motor2 ,
        UINT32 encChannel , UINT32 encTeeth , float encGearRatio ) :
        m_shooterMotor1( motor1 ) ,
        m_shooterMotor2( motor2 ) ,
        m_shooterEncoder( encChannel , encTeeth , encGearRatio ) ,
        m_shooterPID( 0.f , 0.f , 0.f , 1.f / maxSpeed , this , this , 0.04f ) ,
        //m_currentPID( 0 ) ,
        m_isShooting( false ) ,
        m_controlMode( Manual ) ,
        m_negativeOutputAllowed( true ) ,
        m_P( m_shooterPID.GetP() ) ,
        m_I( m_shooterPID.GetI() ) ,
        m_D( m_shooterPID.GetD() ) ,
        m_F( m_shooterPID.GetF() ) ,
        m_setpoint( 0.f ) {
    m_shooterPID.SetOutputRange( -1.f , 1.f );
    m_shooterPID.SetTolerance( 0.f );

    m_shooterPID.SetSetpoint( 0.f );

    m_shooterEncoder.start();
    m_shooterPID.Enable();
    start();
}

Shooter::~Shooter() {
    //removeAllPIDConst();
}

void Shooter::start() {
    m_shooterPID.Enable();
    m_shooterPID.SetSetpoint( m_setpoint );

    m_isShooting = true;
}

void Shooter::stop() {
    m_shooterPID.Reset();

    m_isShooting = false;
}

bool Shooter::isShooting() {
    return m_isShooting;
}

bool Shooter::isReady() {
    return std::fabs(getRPM() - m_setpoint) < 100 && m_isShooting;
    //return std::fabs(getRPM() - m_constants[m_currentPID]->setpoint) < 100 && m_isShooting;
}

void Shooter::setRPM( float wheelSpeed ) {
    if ( m_isShooting ) {
        m_setpoint = wheelSpeed;
        m_shooterPID.SetSetpoint( m_setpoint );
    }
#if 0
    if ( m_isShooting ) {
        m_constants[m_currentPID]->setpoint = wheelSpeed;
        m_shooterPID.SetSetpoint( m_constants[m_currentPID]->setpoint );
    }
#endif
}

void Shooter::setScale( float scaleFactor ) {
#if 0
    if ( m_isShooting ) {
        // Limit 'scaleFactor' to a value between 0 and 1 inclusive
        if ( scaleFactor < 0.f ) {
            scaleFactor = 0.f;
        }
        if ( scaleFactor > 1.f ) {
            scaleFactor = 1.f;
        }

        m_constants[m_currentPID]->setpoint = scaleFactor * maxSpeed;
        m_shooterPID.SetSetpoint( m_constants[m_currentPID]->setpoint );
    }
#endif
    if ( m_isShooting ) {
        // Limit 'scaleFactor' to a value between 0 and 1 inclusive
        if ( scaleFactor < 0.f ) {
            scaleFactor = 0.f;
        }
        if ( scaleFactor > 1.f ) {
            scaleFactor = 1.f;
        }

        m_setpoint = scaleFactor * maxSpeed;
        m_shooterPID.SetSetpoint( m_setpoint );
    }
}

float Shooter::getRPM() {
    return m_shooterEncoder.getRPM();
}

float Shooter::getTargetRPM() {
    return m_setpoint;
    //return m_constants[m_currentPID]->setpoint;
}

bool Shooter::negativeOutputAllowed() {
    return m_negativeOutputAllowed;
}

void Shooter::setControlMode( ControlMode mode ) {
#if 0
    m_controlMode = mode;
    PIDConst constants;

    if ( mode == Manual ) {
        m_currentPID = 0;
    }
    else {
        m_currentPID = 1;
    }

    constants = getPIDConst( m_currentPID );
    m_shooterPID.SetPID( constants.P , constants.I , constants.D , constants.F );
#endif
    m_controlMode = mode;

    if ( mode == Manual ) {
        m_shooterPID.SetPID( 0.f , 0.f , 0.f , m_F );
    }
    else {
        m_shooterPID.SetPID( m_P , m_I , m_D , 0.f );
    }
}

Shooter::ControlMode Shooter::getControlMode() {
    return m_controlMode;
}

void Shooter::setPID( float p , float i , float d ) {
    m_P = p;
    m_I = i;
    m_D = d;

    // Updates PID constants for PIDController object internally
    setControlMode( getControlMode() );
}

#if 0
// Adds constants to list of PID constants; returns position of constants
unsigned int Shooter::addPIDConst( PIDConst constants ) {
    sf::Lock tempLock( m_constantsMutex );

    PIDConst* temp = new PIDConst;
    *temp = constants;
    m_constants.push_back( temp );

    return m_constants.size() - 1;
}

// Removes constants from list of PID constants
void Shooter::removeAllPIDConst() {
    m_constantsMutex.lock();

    while ( m_constants.size() > 0 ) {
        delete m_constants[0];
        m_constants.erase( 0 );
    }

    m_constantsMutex.unlock();
}

// Sets a set of PID constants
void Shooter::setPIDConst( unsigned int index , PIDConst constants ) {
    *m_constants[index] = constants;

    // Updates PID constants for PIDController object internally
    setControlMode( getControlMode() );
}

// Retrieves set of PID constants
PIDConst Shooter::getPIDConst( unsigned int index ) {
    return *m_constants[index];
}

void Shooter::setSetpoint( float setpoint ) {
    m_constants[m_currentPID]->setpoint = setpoint;
    m_shooterPID.SetSetpoint( setpoint );
}

void Shooter::setCurrentPID( unsigned int index ) {
    m_currentPID = index;

    // Updates PID constants for PIDController object internally
    setControlMode( getControlMode() );
}
#endif

double Shooter::PIDGet() {
    if ( m_shooterEncoder.getRPM() < 100.f && m_negativeOutputAllowed ) {
        m_shooterPID.SetOutputRange( 0.f , 1.f );
        m_negativeOutputAllowed = false;
    }
    else if ( m_shooterEncoder.getRPM() >= 100.f && !m_negativeOutputAllowed ) {
        m_shooterPID.SetOutputRange( -1.f , 1.f );
        m_negativeOutputAllowed = true;
    }

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
#if 0
        m_constantsMutex.lock();
        float sSetPoint = m_constants[m_currentPID]->setPoint;
        m_constantsMutex.unlock();
#endif

        if ( m_shooterEncoder.getRPM() >= m_setpoint ) {
        //if ( m_shooterEncoder.getRPM() >= sSetPoint ) {
            m_shooterMotor1.Set( 0.f );
            m_shooterMotor2.Set( 0.f );
        }
        else {
            m_shooterMotor1.Set( -1.f );
            m_shooterMotor2.Set( -1.f );
        }

        break;
    }

    /* The only non-zero term in "Manual" is F, which turns off error
     * correction and responds only to the input given by the user.
     */
    case Manual: {
        m_shooterMotor1.Set( -output );
        m_shooterMotor2.Set( -output );

        break;
    }
    }
}
