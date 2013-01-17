//=============================================================================
//File Name: MecanumDrive.cpp
//Description: Drives the robot with Mecanum wheels in Tank drive configuration
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "MecanumDrive.hpp"
#include <SpeedController.h>
#include <CANJaguar.h>
#include <cmath>

/* We define our own hypotenuse function because the one on VxWorks 6.3 isn't
 * defined
 */
double hypot2( double x , double y ) {
    return sqrt( x * x + y * y );
}

MecanumDrive::MecanumDrive(UINT32 leftMotorChannel, UINT32 rightMotorChannel) :
RobotDrive(leftMotorChannel, rightMotorChannel) {

}

MecanumDrive::MecanumDrive(UINT32 frontLeftMotorChannel, UINT32 rearLeftMotorChannel,
            UINT32 frontRightMotorChannel, UINT32 rearRightMotorChannel) :
            RobotDrive(frontLeftMotorChannel, rearLeftMotorChannel,
                    frontRightMotorChannel, rearRightMotorChannel) {

}

MecanumDrive::MecanumDrive(SpeedController *leftMotor, SpeedController *rightMotor) :
    RobotDrive(leftMotor, rightMotor) {

}

MecanumDrive::MecanumDrive(SpeedController &leftMotor, SpeedController &rightMotor) :
    RobotDrive(leftMotor, rightMotor) {

}

MecanumDrive::MecanumDrive(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
            SpeedController *frontRightMotor, SpeedController *rearRightMotor) :
            RobotDrive(frontLeftMotor, rearLeftMotor,
                    frontRightMotor, rearRightMotor) {

}

MecanumDrive::MecanumDrive(SpeedController &frontLeftMotor, SpeedController &rearLeftMotor,
            SpeedController &frontRightMotor, SpeedController &rearRightMotor) :
            RobotDrive(frontLeftMotor, rearLeftMotor,
                    frontRightMotor, rearRightMotor) {

}

/**
 * Drive method for Mecanum wheeled robots.
 *
 * A method for driving with Mecanum wheeled robots. There are 4 wheels
 * on the robot, arranged so that the front and back wheels are toed in 45 degrees.
 * When looking at the wheels from the top, the roller axles should form an X across the robot.
 *
 * This is designed to be directly driven by joystick axes.
 *
 * @param x The speed that the robot should drive in the X direction. [-1.0..1.0]
 * @param y The speed that the robot should drive in the Y direction.
 * This input is inverted to match the forward == -1.0 that joysticks produce. [-1.0..1.0]
 * @param rotation The rate of rotation for the robot that is completely independent of
 * the translation. [-1.0..1.0]
 * @param gyroAngle The current angle reading from the gyro.  Use this to implement field-oriented controls.
 */
void MecanumDrive::Drive(float x , float y , float rotation , float gyroAngle ) {
    double xIn = x;
    double yIn = y;

    // Compenstate for gyro angle
    RotateVector( xIn , yIn , gyroAngle );

    double wheelSpeeds[kMaxNumberOfMotors];
    /* adding "pi / 4" to the angle rotates the force vector for inline mecanum
     * wheels with the internal rollers rotated "pi / 4" radians
     */

    /* "pi / 2" is added to the angle to rotate sin(0) so it corresponds with
     * driving forward
     */
    wheelSpeeds[kFrontLeftMotor] = hypot2( xIn , yIn ) * sin( atan2( yIn , xIn ) + 3.f * 3.141592f / 4.f ) + rotation;
    wheelSpeeds[kFrontRightMotor] = -hypot2( xIn , yIn ) * cos( atan2( yIn , xIn ) + 3.f * 3.141592f / 4.f ) + rotation;
    wheelSpeeds[kRearLeftMotor] = hypot2( xIn , yIn ) * cos( atan2( yIn , xIn ) + 3.f * 3.141592f / 4.f ) + rotation;
    wheelSpeeds[kRearRightMotor] = -hypot2( xIn , yIn ) * sin( atan2( yIn , xIn ) + 3.f * 3.141592f / 4.f ) + rotation;

    Normalize( wheelSpeeds );

    UINT8 syncGroup = 0x80;

    m_frontLeftMotor->Set( wheelSpeeds[kFrontLeftMotor] * m_invertedMotors[kFrontLeftMotor] * m_maxOutput , syncGroup );
    m_frontRightMotor->Set( wheelSpeeds[kFrontRightMotor] * m_invertedMotors[kFrontRightMotor] * m_maxOutput , syncGroup );
    m_rearLeftMotor->Set( wheelSpeeds[kRearLeftMotor] * m_invertedMotors[kRearLeftMotor] * m_maxOutput , syncGroup );
    m_rearRightMotor->Set( wheelSpeeds[kRearRightMotor] * m_invertedMotors[kRearRightMotor] * m_maxOutput , syncGroup );

    CANJaguar::UpdateSyncGroup(syncGroup);

    m_safetyHelper->Feed();
}

MecanumDrive::~MecanumDrive() {

}
