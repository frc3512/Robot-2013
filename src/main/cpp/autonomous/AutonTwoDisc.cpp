// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

void Robot::AutonTwoDisc() {
    SetShooterAngle(ShooterAngle::kHigh);

    m_shooter.Enable();
    m_shooter.SetReference(Shooter::kMaxSpeed);

    frc2::Wait(7_s);

    // Initialize variables needed for feeding frisbees properly
    auto feedTimeStart = m_autoTime.Get();
    unsigned int shot = 0;

    // Feed frisbees into shooter with a small delay between each
    while (IsAutonomous()) {
        if (m_autoTime.Get() - feedTimeStart > 1.4_s && shot <= 2 &&
            !m_feeder.IsFeeding()) {
            m_feeder.Activate();
            shot++;

            feedTimeStart = m_autoTime.Get();
        }

        m_feeder.Update();
        m_shooter.Update();

        frc2::Wait(0.05_s);
    }
}
