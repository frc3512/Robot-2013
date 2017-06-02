// Copyright (c) 2013-2017 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <stdint.h>

#include <string>

#include "ButtonTracker.hpp"

float ScaleValue(float value) {
    // CONSTANT^-1 is step value (now 1/500)
    return floorf(500.f * (1.f - value) / 2.f) / 500.f;
}

// Creates a wider band in which the joystick won't make the robot move
double deadband(double value) {
    if (std::fabs(value) < 0.05) {
        return 0;
    }

    return value;
}

Robot::Robot()
    : Settings("RobotSettings.txt"),

      driveStick(1),
      shootStick(2),

      mainCompressor(1, 2),

      frisbeeFeeder(1, 3, 0.3f, 0.3f),

      shooterAngle(2),

      flMotor(3),
      rlMotor(5),
      frMotor(7),
      rrMotor(1),
      mainDrive(flMotor, rlMotor, frMotor, rrMotor, 14, 13, 10, 9, 6, 5, 8, 7),

      frisbeeShooter(9, 10, 2, 56, 4.f),

      climbArms(4),
#ifdef KOP_KGYRO
      fieldGyro(1, 1, 2, 0xE5, 0xC4),
#elif defined NEW_KGYRO
      fieldGyro(2, 0x38, 0xE5, 0xC4),
#else
      fieldGyro(1),
#endif

      underGlow(5),

      // Field-oriented driving by default
      isGyroEnabled(true),
      slowRotate(false),
      isShooterManual(false),

      // Create a GraphHost
      pidGraph(3513) {
    pidGraph.resetTime();
    pidGraph.setSendInterval(5);

    driverStation = DriverStationDisplay<Robot>::getInstance(
        std::atoi(Settings::getValueFor("DS_Port").c_str()));

    driverStation->addAutonMethod("CenterMove", &Robot::AutonCenterMove, this);
    driverStation->addAutonMethod("RightMove", &Robot::AutonRightMove, this);
    driverStation->addAutonMethod("LeftMove", &Robot::AutonLeftMove, this);
    driverStation->addAutonMethod("TwoDisc", &Robot::AutonTwoDisc, this);

    // Let motors run for up to 1 second uncontrolled before shutting them down
    mainDrive.SetExpiration(1.f);
    mainDrive.SetDeadband(0.02f);
    frisbeeShooter.setPID(std::atof(getValueFor("PID_SHOOT_P").c_str()),
                          std::atof(getValueFor("PID_SHOOT_I").c_str()),
                          std::atof(getValueFor("PID_SHOOT_D").c_str()));
    frisbeeShooter.updateEncoderFilter(
        std::atof(getValueFor("SHOOTER_RPM_Q").c_str()),
        std::atof(getValueFor("SHOOTER_RPM_R").c_str()));

    frisbeeShooter.stop();
    mainDrive.SquareInputs(true);

    DSpacketTime.Start();
}

void Robot::Autonomous() {
    mainCompressor.Start();

    autoTime.Reset();
    autoTime.Start();

    driverStation->execAutonomous();

    autoTime.Stop();
}

void Robot::OperatorControl() {
    mainCompressor.Start();
    frisbeeShooter.setControlMode(Shooter::PID);

    // Turn on blue underglow
    underGlow.Set(Relay::kOn);
    underGlow.Set(Relay::kForward);

    ButtonTracker driveStickButtons(1);
    ButtonTracker shootStickButtons(2);

    float joyX = 0.f;
    float joyY = 0.f;
    float joyTwist = 0.f;
    float joyGyro = 0.f;

    // Set initial drive mode
    MecanumDrive::DriveMode driveMode = MecanumDrive::Omni;
    mainDrive.SetDriveMode(driveMode);

    // TODO Enable encoders
    // mainDrive.EnableEncoders( true );

    while (IsEnabled() && IsOperatorControl()) {
        DS_PrintOut();

        // update "new" value of joystick buttons
        driveStickButtons.updateButtons();
        shootStickButtons.updateButtons();

        /* ============== Toggle Shooter Motors ============== */
        // turns shooter on/off
        if (shootStickButtons.releasedButton(4)) {
            frisbeeShooter.start();
        } else if (shootStickButtons.releasedButton(5)) {
            frisbeeShooter.stop();
        }

        if (frisbeeShooter.isShooting()) {
            frisbeeShooter.setScale(ScaleValue(shootStick.GetZ()));
        } else {
            frisbeeShooter.stop();
        }

        // Toggle manual RPM setting vs setting with encoder input
        if (shootStickButtons.releasedButton(8)) {
            isShooterManual = false;

            // Reset PID constants
            Settings::update();

            mainDrive.ReloadPID();

            frisbeeShooter.setPID(
                std::atof(getValueFor("PID_SHOOT_P").c_str()),
                std::atof(getValueFor("PID_SHOOT_I").c_str()),
                std::atof(getValueFor("PID_SHOOT_D").c_str()));
            frisbeeShooter.updateEncoderFilter(
                std::atof(getValueFor("SHOOTER_RPM_Q").c_str()),
                std::atof(getValueFor("SHOOTER_RPM_R").c_str()));

            frisbeeShooter.setControlMode(Shooter::PID);
        }

        if (shootStickButtons.releasedButton(9)) {
            isShooterManual = true;

            frisbeeShooter.setControlMode(Shooter::Manual);
        }
        /* =================================================== */

        /* ===== Change shooter angle and speed ===== */
        // Use high shooter angle
        if (shootStickButtons.releasedButton(2)) {
            shooterAngle.Set(true);
        }

        // Use low shooter angle
        if (shootStickButtons.releasedButton(3)) {
            shooterAngle.Set(false);
        }
        /* ========================================== */

        /* ===== Shoot frisbee ===== */
        /* Don't let a frisbee into the shooter if the shooter wheel isn't
         * spinning, except in the case of a manual override.
         */
        if ((frisbeeShooter.getRPM() > 500 && !isShooterManual) ||
            isShooterManual) {
            if (shootStickButtons.releasedButton(1)) {
                frisbeeFeeder.activate();
            }
        }

        // Updates state of feed actuators
        frisbeeFeeder.update();
        /* ========================= */

        /* ===== Control climbing mechanism ===== */
        // Arms up
        if (shootStickButtons.releasedButton(6)) {
            climbArms.Set(true);
        }

        // Arms down
        if (shootStickButtons.releasedButton(7)) {
            climbArms.Set(false);
        }
        /* ====================================== */

        /* ===== Control gyro ===== */
        // Reset gyro
        if (driveStickButtons.releasedButton(8)) {
#if defined(KOP_KGYRO) || defined(NEW_KGYRO)
            fieldGyro.resetXangle(180.f);
#else
            fieldGyro.Reset();
#endif
        }

        if (driveStickButtons.releasedButton(5)) {
            isGyroEnabled = true;

            // kForward turns on blue lights
            underGlow.Set(Relay::kOn);
            underGlow.Set(Relay::kForward);
        }

        if (driveStickButtons.releasedButton(6)) {
            isGyroEnabled = false;

            // kReverse turns on red lights
            underGlow.Set(Relay::kOn);
            underGlow.Set(Relay::kReverse);
        }
        /* ======================== */

        // Enable encoders with PID loops
        if (driveStickButtons.releasedButton(3)) {
            mainDrive.EnableEncoders(true);
        }

        // Disable encoders with PID loops
        if (driveStickButtons.releasedButton(4)) {
            mainDrive.EnableEncoders(false);
        }

        if (isGyroEnabled && driveMode != MecanumDrive::Arcade) {
// Compensate with gyro angle if that's enabled
#if defined(KOP_KGYRO) || defined(NEW_KGYRO)
            joyGyro = fieldGyro.getXangle();
#else
            joyGyro = fieldGyro.GetAngle();
#endif
        } else if (driveMode == MecanumDrive::Arcade) {
            // Compensate for ArcadeDrive robot being backwards
            joyGyro = 90.f;
        } else {
            joyGyro = 0.f;
        }

        // While the thumb button is pressed, don't allow rotation
        if (driveStick.GetRawButton(2)) {
            joyTwist = 0.f;
        } else {
            joyTwist = driveStick.GetZ();
        }

        joyX = driveStick.GetX();
        joyY = driveStick.GetY();

        // Cycle through driving modes
        if (driveStickButtons.releasedButton(7)) {
            if (driveMode == MecanumDrive::Omni) {
                driveMode = MecanumDrive::Strafe;
            } else if (driveMode == MecanumDrive::Strafe) {
                driveMode = MecanumDrive::Arcade;
            } else if (driveMode == MecanumDrive::Arcade) {
                driveMode = MecanumDrive::Omni;
            }

            // Update drive mode
            mainDrive.SetDriveMode(driveMode);
        }

        /* Pivot around a given wheel, or drive normally if no buttons were
         * pressed
         */
        if (driveStick.GetRawButton(9)) {  // FL wheel pivot
            mainDrive.SetDriveMode(MecanumDrive::FLpivot);
        } else if (driveStick.GetRawButton(10)) {  // FR wheel pivot
            mainDrive.SetDriveMode(MecanumDrive::FRpivot);
        } else if (driveStick.GetRawButton(11)) {  // RL wheel pivot
            mainDrive.SetDriveMode(MecanumDrive::RLpivot);
        } else if (driveStick.GetRawButton(12)) {  // RR wheel pivot
            mainDrive.SetDriveMode(MecanumDrive::RRpivot);
        } else {
            mainDrive.SetDriveMode(driveMode);
        }

        // If in lower half, go half speed
        if (ScaleValue(driveStick.GetTwist()) < 0.5) {
            slowRotate = true;
            joyTwist /= 2.f;
        } else {
            slowRotate = false;
        }
        // ^ Goes normal speed if in higher half

        mainDrive.Drive(joyX, joyY, joyTwist, joyGyro);

        Wait(0.05);
    }
}

void Robot::Disabled() {
    mainCompressor.Stop();
    mainDrive.EnableEncoders(false);
    frisbeeShooter.stop();

    ButtonTracker driveStickButtons(1);
    ButtonTracker shootStickButtons(2);

    while (IsDisabled()) {
        DS_PrintOut();

        // Reset gyro
        if (driveStickButtons.releasedButton(8)) {
#if defined(KOP_KGYRO) || defined(NEW_KGYRO)
            fieldGyro.resetXangle(180.f);
#else
            fieldGyro.Reset();
#endif
        }

        // update "new" value of joystick buttons
        driveStickButtons.updateButtons();
        shootStickButtons.updateButtons();

        Wait(0.1);
    }
}

void Robot::DS_PrintOut() {
    if (pidGraph.hasIntervalPassed()) {
        pidGraph.graphData(-mainDrive.GetFRrate(), "FR PID");
        pidGraph.graphData(mainDrive.GetFRsetpoint(), "FR Setpoint");
        pidGraph.graphData(frisbeeShooter.getRPM(), "Shoot Filt RPM");
        pidGraph.graphData(frisbeeShooter.getTargetRPM(), "Shoot Setpoint");
        pidGraph.graphData(mainDrive.GetFLrate(), "FL PID");
        pidGraph.graphData(mainDrive.GetFLsetpoint(), "FL Setpoint");

        pidGraph.resetInterval();
    }

    if (DSpacketTime.HasPeriodPassed(0.2)) {
        /* ===== Print to Driver Station LCD =====
         * Packs the following variables:
         *
         * unsigned int: drive mode
         * int: gyro angle
         * bool: isGyroEnabled
         * bool: slowRotate
         * unsigned int: manual RPM
         * unsigned int: target RPM
         * unsigned int: shooter RPM
         * bool: shooterReady
         * bool: isShooting
         * bool: isShooterManual
         */

        driverStation->clear();

        MecanumDrive::DriveMode driveMode = mainDrive.GetDriveMode();
        std::string strDriveMode;
        if (driveMode == MecanumDrive::Omni) {
            strDriveMode = "Omni";
        } else if (driveMode == MecanumDrive::Strafe) {
            strDriveMode = "Strafe";
        } else if (driveMode == MecanumDrive::Arcade) {
            strDriveMode = "Arcade";
        } else if (driveMode == MecanumDrive::FLpivot) {
            strDriveMode = "FLpivot";
        } else if (driveMode == MecanumDrive::FRpivot) {
            strDriveMode = "FRpivot";
        } else if (driveMode == MecanumDrive::RLpivot) {
            strDriveMode = "RLpivot";
        } else if (driveMode == MecanumDrive::RRpivot) {
            strDriveMode = "RRpivot";
        }
        DS::AddElementData(driverStation, "MODE", strDriveMode);

#if defined(KOP_KGYRO) || defined(NEW_KGYRO)
        DS::AddElementData(driverStation, "GYRO_VAL",
                           static_cast<int32_t>(fieldGyro.getXangle()));
// #elif defined NEW_KGYRO
//        DS::AddElementData( driverStation , "GYRO_VAL" , static_cast<int32_t>(
//        fieldGyro.getXangle() ) );
#else
        DS::AddElementData(driverStation, "GYRO_VAL",
                           static_cast<int32_t>(fieldGyro.GetAngle()));
#endif
        DS::AddElementData(driverStation, "GYRO_ON", isGyroEnabled);

        DS::AddElementData(driverStation, "ROTATE", slowRotate);

        DS::AddElementData(driverStation, "RPM_MAN_DISP",
                           100.f * ScaleValue(shootStick.GetZ()));
        DS::AddElementData(
            driverStation, "RPM_MAN",
            static_cast<int8_t>(100.f * ScaleValue(shootStick.GetZ())));

        DS::AddElementData(driverStation, "RPM_SET_DISP",
                           frisbeeShooter.getTargetRPM());
        DS::AddElementData(driverStation, "RPM_SET",
                           static_cast<int8_t>(frisbeeShooter.getTargetRPM() /
                                               Shooter::maxSpeed * 100.f));

        DS::AddElementData(driverStation, "RPM_REAL_DISP",
                           frisbeeShooter.getRPM());
        DS::AddElementData(driverStation, "RPM_REAL",
                           static_cast<int8_t>(frisbeeShooter.getRPM() /
                                               Shooter::maxSpeed * 100.f));

        DS::AddElementData(driverStation, "SHOOT_READY",
                           frisbeeShooter.isReady());

        DS::AddElementData(driverStation, "SHOOT_ON",
                           frisbeeShooter.isShooting());

        DS::AddElementData(driverStation, "SHOOT_MAN", isShooterManual);

        DS::AddElementData(driverStation, "ARMS_DOWN", !climbArms.Get());

        driverStation->sendToDS();
    }

    driverStation->receiveFromDS();
    /* ====================================== */
}

START_ROBOT_CLASS(Robot);
