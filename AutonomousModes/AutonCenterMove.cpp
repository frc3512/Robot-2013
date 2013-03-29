//=============================================================================
//File Name: AutonCenterMove.cpp
//Description: Drives through pyramid towards the goal and shoots
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "../OurRobot.hpp"

// autoTime is handled from within the main Autonomous call in Autonomous.cpp

void OurRobot::AutonCenterMove() {
    mainDrive.EnableEncoders( true );
    mainDrive.ResetEncoders();

    shooterAngle.Set( true );

    // Start shooter
    frisbeeShooter.start();
    frisbeeShooter.setControlMode( Shooter::Manual );
    frisbeeShooter.setRPM( Shooter::maxSpeed );

    // Move robot 5 meters forward
    while ( IsAutonomous() && mainDrive.GetFLdist() / std::sqrt( 2 ) < 45.f ) {
        DS_PrintOut();

        mainDrive.Drive( 0.8f , 0.f , 0.f , 0.f );

        Wait( 0.1 );
    }

    // Stop and start rotating
    mainDrive.Drive( 0.f , 0.f , 0.f , 0.f );

    float turnTimeStart = autoTime.Get();
    while ( autoTime.Get() - turnTimeStart < 0.23f ) {
        DS_PrintOut();

        mainDrive.Drive( 0.f , 0.f , -0.5f , 0.f );
    }

    // Stop and start shooting
    mainDrive.Drive( 0.f , 0.f , 0.f , 0.f );

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
