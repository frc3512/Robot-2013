//=============================================================================
//File Name: AutonCenterMove.cpp
//Description: Drives around left side of pyramid towards the goal and shoots
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
    //frisbeeShooter.setRPM( 5000.f );
    frisbeeShooter.setScale( 1.f );

    // Move robot 5 meters forward
    while ( IsAutonomous() && mainDrive.GetFLdist() / std::sqrt( 2 ) < 35.f ) {
        DS_PrintOut();

        mainDrive.Drive( 0.8f , 0.f , 0.f , 0.f );
        //frisbeeShooter.setRPM( 5000.f );
        frisbeeShooter.setScale( 1.f );

        Wait( 0.1 );
    }

    // Stop and start rotating
    mainDrive.Drive( 0.f , 0.f , 0.f , 0.f );

    float turnTimeStart = autoTime.Get();
    while ( autoTime.Get() - turnTimeStart < 0.23f ) {
        DS_PrintOut();

        mainDrive.Drive( 0.f , 0.f , -0.5f , 0.f );
        //frisbeeShooter.setRPM( 5000 );
        frisbeeShooter.setScale( 1.f );
    }

    // Stop and start shooting
    mainDrive.Drive( 0.f , 0.f , 0.f , 0.f );

    // Feed frisbees into shooter with a small delay between each
    autoTime.Reset();
    unsigned int shot = 0;
    while ( shot <= 3 && IsAutonomous() ) {
        DS_PrintOut();

        //frisbeeShooter.setRPM( 5000.f );
        frisbeeShooter.setScale( 1.f );

        if ( autoTime.HasPeriodPassed( 1.4f ) ) {
            frisbeeFeeder.activate();
            shot++;
        }

        frisbeeFeeder.update();

        Wait( 0.1 );
    }
}
