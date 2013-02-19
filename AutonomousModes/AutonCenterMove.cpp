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

    while ( IsEnabled() && IsAutonomous() ) {
        DS_PrintOut();

        // Start shooter
        frisbeeShooter.start();
        frisbeeShooter.setScale( 1.f );

        // Move robot 5 meters forward
        while ( IsAutonomous() && mainDrive.GetFLdist() / std::sqrt( 2 ) < 27.f ) {
            mainDrive.Drive( 1.f , 0.f , 0.f , 7.f );
            frisbeeShooter.setScale( 1.f );
        }

        // Stop and shoot frisbees
        mainDrive.Drive( 0.f , 0.f , 0.f , 0.f );

        // Feed frisbees into shooter with a small delay between each
        autoTime.Reset();
        unsigned int shot = 0;
        while ( shot < 3 && IsAutonomous() ) {
            frisbeeShooter.setScale( 1.f );

            if ( autoTime.HasPeriodPassed( 1.f ) ) {
                frisbeeFeeder.activate();
                shot++;
            }

            frisbeeFeeder.update();
        }
    }
}
