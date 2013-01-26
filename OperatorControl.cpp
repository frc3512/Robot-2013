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
    ButtonTracker cameraStickButtons( 3 );

    while ( IsEnabled() && IsOperatorControl() ) {
        //DS_PrintOut(); // TODO Fix packet data

        // update "new" value of joystick buttons
        driveStick1Buttons.updateButtons();
        driveStick2Buttons.updateButtons();
        cameraStickButtons.updateButtons();

        /* ================= Target Selection ================ */
        // selecting target to left of current target
        if ( cameraStickButtons.releasedButton( 4 ) ) {
            turretKinect.setTargetSelect( -1 );

            turretKinect.send();
        }

        // selecting target to right of current target
        if ( cameraStickButtons.releasedButton( 5 ) ) {
            turretKinect.setTargetSelect( 1 );

            turretKinect.send();
        }
        /* =================================================== */

        // Reset gyro
        if ( driveStick1Buttons.releasedButton( 3 ) ) {
            testGyro.Reset();
        }

        /*if ( driveStick1Buttons.releasedButton( 8 ) ) {
            Timer timer;
            timer.Start();

            while ( timer.Get() < 3 ) {
                mainDrive.Drive( 0.f , -1.f , 0.f );
            }

            timer.Stop();
        }

        if ( driveStick1Buttons.releasedButton( 9 ) ) {
            Timer timer;
            timer.Start();

            while ( timer.Get() < 3 ) {
                mainDrive.Drive( 0.f , 1.f , 0.f );
            }

            timer.Stop();
        }*/

        // Aim camera X
        camXTilt.Set( camXTilt.Get() + cameraStick.GetX() / 25.f );

        // Aim camera Y
        camYTilt.Set( camYTilt.Get() - cameraStick.GetY() / 25.f );

        // Aim camera to shoot
        if ( cameraStickButtons.releasedButton( 4 ) ) {
            camXTilt.Set( 1 );
            camYTilt.Set( -1 );
        }

        // Aim camera to feed
        if ( cameraStickButtons.releasedButton( 5 ) ) {
            camXTilt.Set( -1 );
            camYTilt.Set( 1 );
        }

        // move robot based on two joystick inputs
        mainDrive.Drive( driveStick1.GetX() , driveStick1.GetY() , driveStick1.GetZ() , 0 );

        DriverStationLCD::GetInstance()->Clear();
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line1 , 1 , "Gyro: %f" , testGyro.GetAngle() );
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line2 , 1 , "JoyX: %f" , driveStick1.GetX() );
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line3 , 1 , "JoyY: %f" , driveStick1.GetY() );
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line4 , 1 , "JoyTwist: %f" , driveStick1.GetZ() );
        DriverStationLCD::GetInstance()->UpdateLCD();

        Wait( 0.1 );
    }
}
