// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/Relay.h>
#include <frc/Solenoid.h>
#include <frc/Talon.h>
#include <frc/TimedRobot.h>
#include <frc/drive/MecanumDrive.h>
#include <frc2/Timer.h>
#include <wpi/raw_ostream.h>

#include "AutonomousChooser.hpp"
#include "subsystems/Feeder.hpp"
#include "subsystems/Shooter.hpp"

class Robot : public frc::TimedRobot {
public:
    enum class ShooterAngle { kHigh, kLow };
    enum class UnderglowColor { kBlue, kRed, kOff };

    Robot();

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void AutonCenterMove();
    void AutonRightMove();
    void AutonLeftMove();
    void AutonTwoDisc();

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;

    void SetShooterAngle(ShooterAngle angle);
    void SetUnderglowColor(UnderglowColor color);

private:
    frc::AnalogGyro m_gyro{0};

    frc::Joystick m_driveStick{1};
    frc::Joystick m_shootStick{2};

    frc::Solenoid m_shooterAngle{2};
    frc::Solenoid m_climbArms{4};

    frc::Talon m_flMotor{3};
    frc::Talon m_rlMotor{5};
    frc::Talon m_frMotor{7};
    frc::Talon m_rrMotor{1};
    frc::Encoder m_flEncoder{14, 13, true};
    frc::Encoder m_frEncoder{10, 9, true};
    frc::Encoder m_rlEncoder{6, 5, true};
    frc::Encoder m_rrEncoder{8, 7, true};
    frc::MecanumDrive m_drive{m_flMotor, m_frMotor, m_rlMotor, m_rrMotor};

    frc::Relay m_underGlow{5};

    Feeder m_feeder;
    Shooter m_shooter;

    // Field-oriented driving by default
    bool m_isGyroEnabled = true;

    // Used for timing in all Autonomous routines
    AutonomousChooser m_autonChooser{
        "No-op", [] { wpi::outs() << "No-op autonomous\n"; }, [] {}};
    frc2::Timer m_autoTime;
};
