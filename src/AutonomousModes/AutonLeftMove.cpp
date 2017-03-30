// Copyright (c) FRC Team 3512, Spartatroniks 2013-2017. All Rights Reserved.

#include "../Robot.hpp"

// autoTime is handled from within the main Autonomous call in Autonomous.cpp

void Robot::AutonLeftMove() {
    mainDrive.EnableEncoders(true);
    mainDrive.ResetEncoders();

    shooterAngle.Set(true);

    // Start shooter
    frisbeeShooter.start();
    frisbeeShooter.setControlMode(Shooter::Manual);
    frisbeeShooter.setRPM(Shooter::maxSpeed);

    // Move robot 5 meters forward
    while (IsAutonomous() && mainDrive.GetFLdist() / std::sqrt(2) < 45.f) {
        DS_PrintOut();

        mainDrive.Drive(0.8f, 0.f, 0.f, 0.f);

        Wait(0.1);
    }

    // Stop and start rotating to the right
    mainDrive.Drive(0.f, 0.f, 0.f, 0.f);

    float turnTimeStart = autoTime.Get();
    while (autoTime.Get() - turnTimeStart < 0.1f) {
        DS_PrintOut();

        mainDrive.Drive(0.f, 0.f, 0.5f, 0.f);
    }

    // Stop and start shooting
    mainDrive.Drive(0.f, 0.f, 0.f, 0.f);

    // Initialize variables needed for feeding frisbees properly
    double feedTimeStart = autoTime.Get();
    unsigned int shot = 0;

    Wait(3.f);

    // Feed frisbees into shooter with a small delay between each
    while (IsAutonomous()) {
        DS_PrintOut();

        if (autoTime.Get() - feedTimeStart > 1.4 && shot <= 3 &&
            !frisbeeFeeder.isFeeding()) {
            frisbeeFeeder.activate();
            shot++;

            feedTimeStart = autoTime.Get();
        }

        frisbeeFeeder.update();

        Wait(0.05);
    }
}
