//=============================================================================
//File Name: AutonCenter.cpp
//Description: Drives around left side of pyramid towards the goal and shoots
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "../OurRobot.hpp"

// autoTime is handled from within the main Autonomous call in Autonomous.cpp

void OurRobot::AutonCenter() {
    mainDrive.EnableEncoders( true );
    mainDrive.ResetEncoders();

    shooterAngle.Set( true );

    // Start shooter
    frisbeeShooter.start();
    frisbeeShooter.setRPM( 2300.f );

    // Feed frisbees into shooter with a small delay between each
    autoTime.Reset();
    unsigned int shot = 0;
    while ( shot <= 3 && IsAutonomous() ) {
        DS_PrintOut();

        frisbeeShooter.setRPM( 2300.f );

        if ( autoTime.HasPeriodPassed( 1.f ) ) {
            frisbeeFeeder.activate();
            shot++;
        }

        frisbeeFeeder.update();
    }
}
