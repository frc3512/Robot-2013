//=============================================================================
//File Name: OurRobot.hpp
//Description: Main robot class in which all robot sensors and devices are
//             declared
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef OUR_ROBOT_HPP
#define OUR_ROBOT_HPP

#include <cmath>

#include <sys/times.h>

#include <SimpleRobot.h>
#include "Settings.hpp"

#include "AutonContainer.hpp"

#include <Joystick.h>

#include <Compressor.h>
#include <Solenoid.h>

#include "Subsystems/Feeder.hpp"

#include <Talon.h>
#include "Subsystems/MecanumDrive.hpp"

#include "Subsystems/Shooter.hpp"

#include <Victor.h>

#include <Relay.h>

#include <Timer.h>

#include <DriverStationLCD.h>
#include <Gyro.h>
#include <Servo.h>

#include "LiveGrapherHost/graphhost-p.hpp"

// Scale value from -1 .. 1 to 0 .. 1, but within same range of movement
float ScaleValue( float value );

// Adds deadband to joystick value
double deadband( double value );

class DriverStationDisplay;

class OurRobot : public SimpleRobot , public Settings {
public:
    DriverStationDisplay* driverStation; // used for sending data to the Driver Station

    OurRobot();
    ~OurRobot();

    void Autonomous();
    void AutonCenter();
    void AutonCenterMove();
    void AutonLeftShoot();
    void AutonRightShoot();

    void OperatorControl();
    void Disabled();

private:
    uint32_t lastTime;
    uint32_t startTime;

    AutonContainer<OurRobot> autonModes;

    Joystick driveStick;
    Joystick shootStick;

    Compressor mainCompressor;

    Feeder frisbeeFeeder;

    Solenoid shooterAngle;

    Talon flMotor;
    Talon rlMotor;
    Talon frMotor;
    Talon rrMotor;
    MecanumDrive mainDrive;

    Shooter frisbeeShooter;

    Relay leftClimbArm;
    Relay rightClimbArm;

    Gyro testGyro;

    Relay underGlow;

    // Misc. control variables
    bool isGyroEnabled;
    bool slowRotate; // True when slow robot rotation is enabled
    bool isShooterManual;

    // Determines which autonomous mode is run
    char autonMode;

    // Used for timing in all Autonomous routines
    Timer autoTime;

    // The LiveGrapher host
    //GraphHost pidGraph;

    void DS_PrintOut(); // prints messages to driverStation LCD
};

#endif // OUR_ROBOT_HPP
