//=============================================================================
//File Name: OperatorControl.cpp
//Description: Handles driver controls for robot
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include <Timer.h>
#include "OurRobot.hpp"
#include "ButtonTracker.hpp"

void OurRobot::OperatorControl() {
    mainCompressor.Start();

    ButtonTracker driveStick1Buttons( 1 );
    ButtonTracker driveStick2Buttons( 2 );
    ButtonTracker turretStickButtons( 3 );

    while ( IsEnabled() && IsOperatorControl() ) {
        //DS_PrintOut(); // TODO Fix packet data

        // update "new" value of joystick buttons
        driveStick1Buttons.updateButtons();
        driveStick2Buttons.updateButtons();
        turretStickButtons.updateButtons();

        /* ================= Target Selection ================ */
        // selecting target to left of current target
        if ( turretStickButtons.releasedButton( 4 ) ) {
            turretKinect.setTargetSelect( -1 );

            turretKinect.send();
        }

        // selecting target to right of current target
        if ( turretStickButtons.releasedButton( 5 ) ) {
            turretKinect.setTargetSelect( 1 );

            turretKinect.send();
        }
        /* =================================================== */

        // move robot based on two joystick inputs
        mainDrive.Drive( driveStick1.GetX() , driveStick1.GetY() , driveStick2.GetX() , 0 );
        //mainDrive.Drive( ScaleZ( driveStick1 ) * driveStick1.GetY() , ScaleZ( driveStick2 ) * driveStick2.GetX() , false );

        Wait( 0.1 );
    }
}
