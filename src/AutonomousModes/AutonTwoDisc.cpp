// Copyright (c) FRC Team 3512, Spartatroniks 2013-2017. All Rights Reserved.

#include "../Robot.hpp"

// autoTime is handled from within the main Autonomous call in Autonomous.cpp

void Robot::AutonTwoDisc() {
    shooterAngle.Set(true);

    // Start shooter
    frisbeeShooter.start();
    frisbeeShooter.setControlMode(Shooter::Manual);
    frisbeeShooter.setRPM(Shooter::maxSpeed);

    Wait(7.f);

    // Initialize variables needed for feeding frisbees properly
    double feedTimeStart = autoTime.Get();
    unsigned int shot = 0;

    // Feed frisbees into shooter with a small delay between each
    while (IsAutonomous()) {
        DS_PrintOut();

        if (autoTime.Get() - feedTimeStart > 1.4 && shot <= 2 &&
            !frisbeeFeeder.isFeeding()) {
            frisbeeFeeder.activate();
            shot++;

            feedTimeStart = autoTime.Get();
        }

        frisbeeFeeder.update();

        Wait(0.05);
    }
}
