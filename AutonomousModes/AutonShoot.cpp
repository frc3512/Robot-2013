//=============================================================================
//File Name: AutonShoot.cpp
//Description: Shoots balls contained within robot at hoop
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "../OurRobot.hpp"

// autoTime is handled from within the main Autonomous call in Autonomous.cpp

void OurRobot::AutonShoot() {
    turretKinect.clearValues();

    mainDrive.EnableEncoders( true );

    while ( IsEnabled() && IsAutonomous() ) {
        //DS_PrintOut(); // TODO Fix packet data
        //mainDrive


        Wait( 0.1 );
    }
}
