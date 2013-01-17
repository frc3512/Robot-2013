//=============================================================================
//File Name: MecanumDrive.hpp
//Description: Drives the robot with Mecanum wheels in Tank drive configuration
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef MECANUM_DRIVE_HPP
#define MECANUM_DRIVE_HPP

#include "RobotDrive.h"

class MecanumDrive : public RobotDrive {
public:
    MecanumDrive(UINT32 leftMotorChannel, UINT32 rightMotorChannel);
    MecanumDrive(UINT32 frontLeftMotorChannel, UINT32 rearLeftMotorChannel,
                UINT32 frontRightMotorChannel, UINT32 rearRightMotorChannel);
    MecanumDrive(SpeedController *leftMotor, SpeedController *rightMotor);
    MecanumDrive(SpeedController &leftMotor, SpeedController &rightMotor);
    MecanumDrive(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
                SpeedController *frontRightMotor, SpeedController *rearRightMotor);
    MecanumDrive(SpeedController &frontLeftMotor, SpeedController &rearLeftMotor,
                SpeedController &frontRightMotor, SpeedController &rearRightMotor);

    /* This function assumes field-oriented driving
     * x is the magnitude of x translation of the robot [-1..1]
     * y is the magnitude of y translation of the robot [-1..1]
     * rotation is the magnitude of rotation [-1..1]
     */
    void Drive( float x , float y , float rotation , float gyroAngle = 0.0 );

    virtual ~MecanumDrive();

    DISALLOW_COPY_AND_ASSIGN(MecanumDrive);
};

#endif // MECANUM_DRIVE_HPP
