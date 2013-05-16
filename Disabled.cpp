//=============================================================================
//File Name: Disabled.cpp
//Description: This function runs when robot is in disabled state
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include <Timer.h>
#include "OurRobot.hpp"
#include "ButtonTracker.hpp"

void OurRobot::Disabled() {
    mainCompressor.Stop();
    frisbeeShooter.stop();

    ButtonTracker driveStickButtons( 1 );
    ButtonTracker shootStickButtons( 2 );

    while ( IsDisabled() ) {
        DS_PrintOut();

        // Reset gyro
        if ( driveStickButtons.releasedButton( 8 ) ) {
#ifdef NEW_GYRO
            fieldGyro.resetXangle( 180.f );
#else
            fieldGyro.Reset();
#endif
        }

        // update "new" value of joystick buttons
        driveStickButtons.updateButtons();
        shootStickButtons.updateButtons();

        Wait( 0.1 );
    }
}
