//=============================================================================
//File Name: Shooter.hpp
//Description: Provides an interface to the dual-motored shooter and its
//             encoder; it also has an interface for a PID loop
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef SHOOTER_HPP
#define SHOOTER_HPP

#include <PIDSource.h>
#include <PIDOutput.h>
#include <PIDController.h>
#include <Victor.h>
#include "../GeartoothEncoder.hpp"

class Shooter : public PIDSource , public PIDOutput {
public:
    static const float maxSpeed;

    Shooter( UINT32 motor1 , UINT32 motor2 ,
            UINT32 encChannel , UINT32 encTeeth , float encGearRatio );
    virtual ~Shooter();

    typedef enum {
        PID = 0,
        BangBang
    } ControlMode;

    void start();
    void stop();
    bool isShooting();

    // Returns true when the shooter RPM is on target
    bool isReady();

    // Set shooter to the given RPM
    void setRPM( float wheelSpeed );

    // Returns current RPM of encoder
    float getRPM();

    /* Set shooter to a fraction of the maximum speed [0..1] (overrides control
     * loops)
     */
    void setScale( float scaleFactor );

    // Get setpoint of PID loop
    float getTargetRPM();

    // Enables control loops like PID
    void enableControl();

    // Disables control loops like PID
    void disableControl();

    void setControlMode( ControlMode mode );
    ControlMode getControlMode();

private:
    Victor m_shooterMotor1;
    Victor m_shooterMotor2;

    GeartoothEncoder m_shooterEncoder;

    PIDController m_shooterPID;
    float m_setpoint;

    bool m_isShooting;
    bool m_reversed;

    ControlMode m_controlMode;

    // Used by PID controller to get RPM
    double PIDGet();

    // PID controller output makes shooter wheels move
    void PIDWrite( float output );
};

#endif // SHOOTER_HPP
