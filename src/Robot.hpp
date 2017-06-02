// Copyright (c) 2013-2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cmath>

#include <Compressor.h>
#include <DriverStationLCD.h>
#include <Joystick.h>
#include <Relay.h>
#include <SimpleRobot.h>
#include <Solenoid.h>
#include <Talon.h>
#include <Timer.h>
#include <Victor.h>

#include "DriverStationDisplay.hpp"
#include "Settings.hpp"
#include "Subsystems/Feeder.hpp"
#include "Subsystems/MecanumDrive.hpp"
#include "Subsystems/Shooter.hpp"

// #define KOP_KGYRO
#ifdef KOP_KGYRO
#include "KalmanGyro/KOP_ADXL345.hpp"
#elif defined NEW_KGYRO
#include "KalmanGyro/ITG3200_ADXL345.hpp"
#else
#include <Gyro.h>
#endif

#include "LiveGrapherHost/GraphHost.hpp"

// Scale value from -1 .. 1 to 0 .. 1, but within same range of movement
float ScaleValue(float value);

// Adds deadband to joystick value
double deadband(double value);

class Robot : public SimpleRobot, public Settings {
public:
    DriverStationDisplay<Robot>*
        driverStation;  // used for sending data to the Driver Station

    Robot();

    void Autonomous();
    void AutonCenterMove();
    void AutonRightMove();
    void AutonLeftMove();
    void AutonTwoDisc();

    void OperatorControl();
    void Disabled();

private:
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

    Solenoid climbArms;

#ifdef KOP_KGYRO
    KOP_ADXL345 fieldGyro;
#elif defined NEW_KGYRO
    ITG3200_ADXL345 fieldGyro;
#else
    Gyro fieldGyro;
#endif

    Relay underGlow;

    // Misc. control variables
    bool isGyroEnabled;
    bool slowRotate;  // True when slow robot rotation is enabled
    bool isShooterManual;

    // Used for timing in all Autonomous routines
    Timer autoTime;

    // Used for determining when to send DSDisplay packets
    Timer DSpacketTime;

    // The LiveGrapher host
    GraphHost pidGraph;

    void DS_PrintOut();  // prints messages to driverStation LCD
};
