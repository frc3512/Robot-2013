//=============================================================================
//File Name: AutonCenterShoot.cpp
//Description: Shoots at goals from back of pyramid
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "../OurRobot.hpp"

// autoTime is handled from within the main Autonomous call in Autonomous.cpp

void OurRobot::AutonCenterShoot() {
    mainDrive.EnableEncoders( true );
    mainDrive.ResetEncoders();

    shooterAngle.Set( true );

    // Start shooter
    frisbeeShooter.start();
    frisbeeShooter.setRPM( 2300.f );

    // Initialize variables needed for feeding frisbees properly
    double feedTimeStart = autoTime.Get();
    unsigned int shot = 0;

    // Feed frisbees into shooter with a small delay between each
    while ( shot <= 3 && IsAutonomous() ) {
        DS_PrintOut();

        if ( autoTime.Get() - feedTimeStart > 1.4 && !frisbeeFeeder.isFeeding() ) {
            frisbeeFeeder.activate();
            shot++;

            feedTimeStart = autoTime.Get();
        }

        frisbeeFeeder.update();

        Wait( 0.1 );
    }
}
