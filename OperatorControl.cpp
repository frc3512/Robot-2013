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

    bool isShooting = false;
    bool shooterIsManual = false;

    // Field-oriented driving by default
    bool gyroEnabled = true;

    float joyX = 0.f;
    float joyY = 0.f;
    float joyTwist = 0.f;
    float joyGyro = 0.f;

    // Set initial drive mode
    MecanumDrive::DriveMode driveMode = MecanumDrive::Omni;
    mainDrive.SetDriveMode( driveMode );

    while ( IsEnabled() && IsOperatorControl() ) {
        DS_PrintOut(); // TODO Fix packet data

        // update "new" value of joystick buttons
        driveStick1Buttons.updateButtons();
        driveStick2Buttons.updateButtons();
        cameraStickButtons.updateButtons();

#if 0
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
#endif

        /* ============== Toggle Shooter Motors ============== */
        // turns shooter on/off
        if ( driveStick2Buttons.releasedButton( 1 ) ) { // if released trigger
            isShooting = !isShooting;
        }

        if ( isShooting ) {
            if ( shooterIsManual ) { // let driver change shooter speed manually
                shooterMotor1.Set( ScaleZ(driveStick2) );
                shooterMotor2.Set( ScaleZ(driveStick2) );
            }
            else { // else adjust shooter voltage to match RPM
                //pidControl.SetTargetDistance( 25.f ); // * 0.00328084f
                //pidControl.Update();

                /*float encoderRPM = 60.f / ( 16.f * shooterEncoder.GetPeriod() );
                if ( encoderRPM >= 72.0 * ScaleZ(turretStick) * 60.0 ) {
                    shooterMotorLeft.Set( 0 );
                    shooterMotorRight.Set( 0 );
                }
                else if ( encoderRPM > 2242.f ) {
                    shooterMotorLeft.Set( -0.3f );
                    shooterMotorRight.Set( 0.3f );
                }
                else {
                    shooterMotorLeft.Set( -1 );
                    shooterMotorRight.Set( 1 );
                }*/
            }
        }
        else {
            shooterMotor1.Set( 0.f );
            shooterMotor2.Set( 0.f );
        }

        // toggle manual RPM setting vs setting with encoder input
        if ( driveStick1Buttons.releasedButton( 12 ) ) {
            shooterIsManual = !shooterIsManual;
        }
        /* =================================================== */

        // Reset gyro
        if ( driveStick2Buttons.releasedButton( 8 ) ) {
            testGyro.Reset();
        }

        // Enable encoders with PID loops
        if ( driveStick2Buttons.releasedButton( 3 ) ) {
            //mainDrive.EnableEncoders( true );
        }

        // Disable encoders with PID loops
        if ( driveStick2Buttons.releasedButton( 4 ) ) {
            //mainDrive.EnableEncoders( false );
        }

        if ( driveStick2Buttons.releasedButton( 5 ) ) {
            gyroEnabled = true;
        }

        if ( driveStick2Buttons.releasedButton( 6 ) ) {
            gyroEnabled = false;
        }

        /*if ( driveStick2Buttons.releasedButton( 8 ) ) {
            Timer timer;
            timer.Start();

            while ( timer.Get() < 3 ) {
                mainDrive.Drive( 0.f , -1.f , 0.f );
            }

            timer.Stop();
        }

        if ( driveStick2Buttons.releasedButton( 9 ) ) {
            Timer timer;
            timer.Start();

            while ( timer.Get() < 3 ) {
                mainDrive.Drive( 0.f , 1.f , 0.f );
            }

            timer.Stop();
        }*/

#if 0
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
#endif

        // Compensate with gyro angle if that's enabled
        if ( gyroEnabled ) {
            joyGyro = testGyro.GetAngle();
        }
        else {
            joyGyro = 0.f;
        }

        // While the thumb button is pressed, don't allow rotation
        if ( driveStick2.GetRawButton( 2 ) ) {
            joyTwist = 0.f;
        }
        else {
            joyTwist = driveStick2.GetZ();
        }

        // Set X and Y joystick inputs
        joyX = driveStick2.GetX();
        joyY = driveStick2.GetY();

        // Cycle through driving modes
        if ( driveStick2Buttons.releasedButton( 7 ) ) {
            if ( driveMode == MecanumDrive::Omni ) {
                driveMode = MecanumDrive::Strafe;
            }
            else if ( driveMode == MecanumDrive::Strafe ) {
                driveMode = MecanumDrive::Arcade;
            }
            else if ( driveMode == MecanumDrive::Arcade ) {
                driveMode = MecanumDrive::Omni;
            }

            // Update drive mode
            mainDrive.SetDriveMode( driveMode );
        }

        /* Pivot around a given wheel, or drive normally if no buttons were
         * pressed
         */
#if 0
        if ( driveStick2.GetRawButton( 9 ) ) { // FL wheel pivot
            mainDrive.SetDriveMode( MecanumDrive::FLpivot );
        }
        else if ( driveStick2.GetRawButton( 10 ) ) { // FR wheel pivot
            mainDrive.SetDriveMode( MecanumDrive::FRpivot );
        }
        else if ( driveStick2.GetRawButton( 11 ) ) { // RL wheel pivot
            mainDrive.SetDriveMode( MecanumDrive::RLpivot );
        }
        else if ( driveStick2.GetRawButton( 12 ) ) { // RR wheel pivot
            mainDrive.SetDriveMode( MecanumDrive::RRpivot );
        }
#endif

        mainDrive.Drive( joyX , joyY , joyTwist , joyGyro );

        Wait( 0.1 );
    }
}
