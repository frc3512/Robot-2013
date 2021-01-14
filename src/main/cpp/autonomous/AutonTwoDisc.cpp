// Copyright (c) 2013-2021 FRC Team 3512. All Rights Reserved.

#include <frc2/Timer.h>

#include "Robot.hpp"

void Robot::AutonTwoDisc() {
    SetShooterAngle(ShooterAngle::kHigh);

    m_shooter.Enable();
    m_shooter.SetReference(Shooter::kMaxSpeed);

    frc2::Timer timer;
    timer.Start();
    while (!timer.HasPeriodPassed(7_s)) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            return;
        }
    }

    // Initialize variables needed for feeding frisbees properly
    auto feedTimeStart = timer.Get();
    unsigned int shot = 0;

    // Feed frisbees into shooter with a small delay between each
    while (shot <= 2) {
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
