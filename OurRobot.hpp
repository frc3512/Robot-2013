//=============================================================================
//File Name: OurRobot.hpp
//Description: Main robot class in which all robot sensors and devices are
//             declared
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef OUR_ROBOT_HPP
#define OUR_ROBOT_HPP

#include <cmath>

#include <SimpleRobot.h>
#include "Settings.hpp"

#include "AutonContainer.hpp"

#include <Compressor.h>
#include <Joystick.h>

#include <Talon.h>
#include "MecanumDrive.hpp"

#include "Kinect/TurretKinect.hpp"

#include <Timer.h>

// Scales joystick's Z axis values from -1 .. 1 to 0 .. 1, but within same range of movement
float ScaleZ( Joystick& stick );

// Adds deadband to joystick value
double deadband( double value );

class DriverStationDisplay;

class OurRobot : public SimpleRobot , public Settings {
public:
    DriverStationDisplay* driverStation; // used for sending data to the Driver Station

    OurRobot();
    ~OurRobot();

    void Autonomous();
    void AutonShoot();
    void AutonFeed();

    void OperatorControl();
    void Disabled();

private:
    AutonContainer<OurRobot> autonModes;

    Compressor mainCompressor;

    Talon flMotor;
    Talon rlMotor;
    Talon frMotor;
    Talon rrMotor;
    MecanumDrive mainDrive;

    Joystick driveStick1;
    Joystick driveStick2;
    Joystick turretStick;

    TurretKinect turretKinect;

    // Determines which autonomous mode is run
    char autonMode;

    // Used for timing in all Autonomous routines
    Timer autoTime;

    void DS_PrintOut(); // prints messages to driverStation LCD
};

#endif // OUR_ROBOT_HPP
