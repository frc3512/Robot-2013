//=============================================================================
//File Name: AutonFeed.cpp
//Description: Reverses ball retrieval mechanism for feeding balls to another
//             team during autonomous
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "../OurRobot.hpp"

void OurRobot::AutonFeed() {
    while ( IsEnabled() && IsAutonomous() ) {
        // DS_PrintOut(); // TODO Fix packet data

        Wait( 0.1 );
    }
}
