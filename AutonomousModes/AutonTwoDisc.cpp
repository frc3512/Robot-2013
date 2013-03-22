//=============================================================================
//File Name: AutonTwoDisc.cpp
//Description: Shoots two discs at the goal (doesn't drive in Autonomous)
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "../OurRobot.hpp"

// autoTime is handled from within the main Autonomous call in Autonomous.cpp

void OurRobot::AutonTwoDisc() {

    shooterAngle.Set( true );

    // Start shooter
    frisbeeShooter.start();
    frisbeeShooter.setControlMode( Shooter::Manual );
    frisbeeShooter.setRPM( Shooter::maxSpeed );

    Wait( 7.f );

    // Initialize variables needed for feeding frisbees properly
    double feedTimeStart = autoTime.Get();
    unsigned int shot = 0;

    // Feed frisbees into shooter with a small delay between each
    while ( shot <= 2 && IsAutonomous() ) {
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
