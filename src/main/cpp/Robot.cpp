// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

double ScaleZ(frc::Joystick& stick) {
    return std::floor(500.0 * (1.0 - stick.GetZ()) / 2.0) /
           500.0;  // CONSTANT^-1 is step value (now 1/500)
}

Robot::Robot() {
    m_flEncoder.SetDistancePerPulse(60.0 / 250.0);
    m_frEncoder.SetDistancePerPulse(60.0 / 250.0);
    m_rlEncoder.SetDistancePerPulse(60.0 / 250.0);
    m_rrEncoder.SetDistancePerPulse(60.0 / 250.0);

    m_autonChooser.AddAutonomous(
        "CenterMove", [=] { AutonCenterMove(); }, [] {});
    m_autonChooser.AddAutonomous(
        "RightMove", [=] { AutonRightMove(); }, [] {});
    m_autonChooser.AddAutonomous(
        "LeftMove", [=] { AutonLeftMove(); }, [] {});
    m_autonChooser.AddAutonomous(
        "TwoDisc", [=] { AutonTwoDisc(); }, [] {});
}

void Robot::AutonomousInit() {
    m_gyro.Reset();

    m_autoTime.Reset();
    m_autoTime.Start();

    m_autonChooser.RunAutonomousInit();
}

void Robot::AutonomousPeriodic() { m_autonChooser.RunAutonomousPeriodic(); }

void Robot::TeleopInit() {
    m_gyro.Reset();
    SetUnderglowColor(UnderglowColor::kBlue);
}

void Robot::TeleopPeriodic() {
    if (m_shootStick.GetRawButtonPressed(4)) {
        m_shooter.Enable();
    } else if (m_shootStick.GetRawButtonPressed(5)) {
        m_shooter.Disable();
    }

    if (m_shooter.IsEnabled()) {
        m_shooter.SetReference(ScaleZ(m_shootStick) * Shooter::kMaxSpeed);
    }

    if (m_shootStick.GetRawButtonPressed(2)) {
        SetShooterAngle(ShooterAngle::kHigh);
    }

    if (m_shootStick.GetRawButtonPressed(3)) {
        SetShooterAngle(ShooterAngle::kLow);
    }

    if (m_shooter.AtReference() && m_shootStick.GetRawButtonPressed(1)) {
        // Shoot frisbee
        m_feeder.Activate();
    }

    // Updates state of feed actuators
    m_feeder.Update();
    m_shooter.Update();

    if (m_shootStick.GetRawButtonPressed(6)) {
        // Climbing arms up
        m_climbArms.Set(true);
    }

    if (m_shootStick.GetRawButtonPressed(7)) {
        // Climbing arms down
        m_climbArms.Set(false);
    }

    if (m_driveStick.GetRawButtonPressed(8)) {
        m_gyro.Reset();
    }

    if (m_driveStick.GetRawButtonPressed(5)) {
        m_isGyroEnabled = true;
        SetUnderglowColor(UnderglowColor::kBlue);
    }

    if (m_driveStick.GetRawButtonPressed(6)) {
        m_isGyroEnabled = false;
        SetUnderglowColor(UnderglowColor::kRed);
    }

    // If in lower half, go half speed
    double joyTwist = m_driveStick.GetTwist();
    if (ScaleZ(m_driveStick) < 0.5) {
        joyTwist /= 2.0;
    }

    if (m_isGyroEnabled) {
        m_drive.DriveCartesian(m_driveStick.GetX(), m_driveStick.GetY(),
                               joyTwist, m_gyro.GetAngle());
    } else {
        m_drive.DriveCartesian(m_driveStick.GetX(), m_driveStick.GetY(),
                               joyTwist);
    }
}

void Robot::DisabledInit() { m_shooter.Disable(); }

void Robot::SetShooterAngle(ShooterAngle angle) {
    if (angle == ShooterAngle::kHigh) {
        m_shooterAngle.Set(true);
    } else if (angle == ShooterAngle::kLow) {
        m_shooterAngle.Set(false);
    }
}

void Robot::SetUnderglowColor(UnderglowColor color) {
    if (color == UnderglowColor::kBlue) {
        m_underGlow.Set(frc::Relay::kOn);
        m_underGlow.Set(frc::Relay::kForward);
    } else if (color == UnderglowColor::kRed) {
        m_underGlow.Set(frc::Relay::kOn);
        m_underGlow.Set(frc::Relay::kReverse);
    } else {
        m_underGlow.Set(frc::Relay::kOff);
    }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
