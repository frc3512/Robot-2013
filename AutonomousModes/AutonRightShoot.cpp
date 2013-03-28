//=============================================================================
//File Name: AutonRightShoot.cpp
//Description: Moves to the right and shoots at goals from back of pyramid
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "../OurRobot.hpp"

// autoTime is handled from within the main Autonomous call in Autonomous.cpp

void OurRobot::AutonRightShoot() {
    mainDrive.EnableEncoders( true );
    mainDrive.ResetEncoders();

    shooterAngle.Set( true );

    // Start shooter
    frisbeeShooter.start();
    frisbeeShooter.setRPM( 2300.f );

    // Move robot 5 meters forward
    while ( IsAutonomous() && mainDrive.GetFLdist() < 35.f ) {
        DS_PrintOut();

        mainDrive.Drive( 0.f , -0.8f , 0.f , 0.f );

        Wait( 0.1 );
    }

#if 0
    // Stop and start rotating to the left
    mainDrive.Drive( 0.f , 0.f , 0.f , 0.f );

    float turnTimeStart = autoTime.Get();
    while ( autoTime.Get() - turnTimeStart < 0.1f ) {
        DS_PrintOut();

        mainDrive.Drive( 0.f , 0.f , 0.5f , 0.f );
    }
#endif

    // Stop and start shooting
    mainDrive.Drive( 0.f , 0.f , 0.f , 0.f );

    // Initialize variables needed for feeding frisbees properly
    double feedTimeStart = autoTime.Get();
    unsigned int shot = 0;

    // Feed frisbees into shooter with a small delay between each
    while ( IsAutonomous() ) {
        DS_PrintOut();

        if ( autoTime.Get() - feedTimeStart > 1.4 && shot <= 3 && !frisbeeFeeder.isFeeding() ) {
            frisbeeFeeder.activate();
            shot++;

            feedTimeStart = autoTime.Get();
        }

        frisbeeFeeder.update();

        Wait( 0.1 );
    }
}
