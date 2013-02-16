//=============================================================================
//File Name: AutonCenter.cpp
//Description: Drives around left side of pyramid towards the goal and shoots
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "../OurRobot.hpp"

// autoTime is handled from within the main Autonomous call in Autonomous.cpp

void OurRobot::AutonCenter() {
    mainDrive.EnableEncoders( true );

    while ( IsEnabled() && IsAutonomous() ) {
        DS_PrintOut();

        // Start shooter
        frisbeeShooter.start();
        frisbeeShooter.setRPM( 2300.f );

        // Move robot 5 meters forward
        while ( mainDrive.GetFLdist() < 5.f ) {
            mainDrive.Drive( 0.f , 1.f , 0.f , 0.f );
            frisbeeShooter.setRPM( 2300.f );
        }

        // Stop and shoot frisbees
        mainDrive.Drive( 0.f , 0.f , 0.f , 0.f );

        // Feed frisbees into shooter with a small delay between each
        autoTime.Reset();
        for ( unsigned int i = 0 ; i < 4 ; i++ ) {
            frisbeeShooter.setRPM( 2300.f );

            if ( autoTime.HasPeriodPassed( 3.f ) ) {
                frisbeeFeeder.activate();
            }
        }
    }
}
