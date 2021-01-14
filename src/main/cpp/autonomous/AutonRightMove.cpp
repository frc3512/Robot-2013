// Copyright (c) 2013-2021 FRC Team 3512. All Rights Reserved.

#include <frc2/Timer.h>

#include "Robot.hpp"

void Robot::AutonRightMove() {
    m_flEncoder.Reset();

    SetShooterAngle(ShooterAngle::kLow);

    m_shooter.Enable();
    m_shooter.SetReference(Shooter::kMaxSpeed);

    // Move robot 5 meters sideways
    while (m_flEncoder.GetDistance() < 35.0) {
        m_drive.DriveCartesian(0.8, 0.0, 0.0, 0.0);

        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            return;
        }
    }

    // Stop and start rotating to the left
    m_drive.DriveCartesian(0.0, 0.0, 0.0, 0.0);

    frc2::Timer timer;
    timer.Start();
    while (!timer.HasPeriodPassed(0.53_s)) {
        m_drive.DriveCartesian(0.0, 0.0, -0.5, 0.0);

        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            return;
        }
    }

    // Stop and start shooting
    m_drive.DriveCartesian(0.0, 0.0, 0.0, 0.0);

    // Initialize variables needed for feeding frisbees properly
    auto feedTimeStart = timer.Get();
    unsigned int shot = 0;

    // Feed frisbees into shooter with a small delay between each
    while (shot <= 3) {
        if (timer.Get() - feedTimeStart > 1.4_s && !m_feeder.IsFeeding()) {
            m_feeder.Activate();
            shot++;

            feedTimeStart = timer.Get();
        }

        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            return;
        }
    }
}
