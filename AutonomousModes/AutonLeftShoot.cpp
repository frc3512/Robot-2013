//=============================================================================
//File Name: AutonLeftShoot.cpp
//Description: Drives around left side of pyramid towards the goal and shoots
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "../OurRobot.hpp"

// autoTime is handled from within the main Autonomous call in Autonomous.cpp

void OurRobot::AutonLeftShoot() {
    mainDrive.EnableEncoders( true );

    while ( IsEnabled() && IsAutonomous() ) {
        DS_PrintOut();

        // Start shooter
        frisbeeShooter.start();
        frisbeeShooter.setRPM( 2300.f );

        // Move robot 5 meters left
        while ( mainDrive.GetFLdist() < std::sqrt( 2.f ) * 5.f ) {
            mainDrive.Drive( -1.f , 0.f , 0.f , 0.f );
        }

        // Move robot forward 5 meters
        while ( mainDrive.GetFLdist() < 5.f ) {
            mainDrive.Drive( 0.f , 1.f , 0.f , 0.f );
        }

        // Stop and shoot frisbees
        mainDrive.Drive( 0.f , 0.f , 0.f , 0.f );

        // Feed frisbees into shooter with a small delay between each
        for ( unsigned int i = 0 ; i < 4 ; i++ ) {
            frisbeeFeeder.activate();
            Wait( 3.f );
        }
    }
}
