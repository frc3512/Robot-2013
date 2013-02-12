//=============================================================================
//File Name: OperatorControl.cpp
//Description: Handles driver controls for robot
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include <Timer.h>
#include "OurRobot.hpp"
#include "ButtonTracker.hpp"
#include <iostream> // TODO Remove me

void OurRobot::OperatorControl() {
    mainCompressor.Start();
    shooterEncoder.Start();

    ButtonTracker driveStick1Buttons( 1 );
    ButtonTracker driveStick2Buttons( 2 );
    ButtonTracker shootStickButtons( 3 );

    bool isShooting = false;
    bool shooterIsManual = true;

    // Field-oriented driving by default
    bool gyroEnabled = true;

    float joyX = 0.f;
    float joyY = 0.f;
    float joyTwist = 0.f;
    float joyGyro = 0.f;

    // Set initial drive mode
    MecanumDrive::DriveMode driveMode = MecanumDrive::Omni;
    mainDrive.SetDriveMode( driveMode );

    mainDrive.EnableEncoders( true );

    while ( IsEnabled() && IsOperatorControl() ) {
        DS_PrintOut(); // TODO Fix packet data

        // update "new" value of joystick buttons
        driveStick1Buttons.updateButtons();
        driveStick2Buttons.updateButtons();
        shootStickButtons.updateButtons();

        /* ============== Toggle Shooter Motors ============== */
        // turns shooter on/off
        if ( shootStickButtons.releasedButton( 4 ) ) {
            isShooting = true;
        }
        else if ( shootStickButtons.releasedButton( 5 ) ) {
            isShooting = false;
        }

        if ( isShooting ) {
            if ( shooterIsManual ) { // let driver change shooter speed manually
                shooterMotor1.Set( -ScaleValue( shootStick.GetAxis(Joystick::kZAxis) ) );
                shooterMotor2.Set( -ScaleValue( shootStick.GetAxis(Joystick::kZAxis) ) );
            }
            else { // else adjust shooter voltage to match RPM
                //pidControl.SetTargetDistance( 25.f ); // * 0.00328084f
                //pidControl.Update();

                /*float encoderRPM = 60.f / ( 16.f * shooterEncoder.GetPeriod() );
                if ( encoderRPM >= 72.0 * ScaleZ(shootStick) * 60.0 ) {
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
        if ( shootStickButtons.releasedButton( 12 ) ) {
            shooterIsManual = !shooterIsManual;
        }
        /* =================================================== */

        /* ===== Change shooter angle and speed ===== */
        // Use high shooter angle
        if ( shootStickButtons.releasedButton( 2 ) ) {
            shooterAngle.Set( true );
        }

        // Use low shooter angle
        if ( shootStickButtons.releasedButton( 3 ) ) {
            shooterAngle.Set( false );
        }
        /* ========================================== */

        /* ===== Shoot frisbee ===== */
        /* A delay is used here so the frisbee feed actuator is fully activated
         * before retracting it again
         */

        if ( shootStickButtons.releasedButton( 1 ) ) {
            // Pull feeder in for pushing frisbee
            frisbeeFeed.Set( true );

            // Lower shooter guard so frisbees can leave
            frisbeeGuard.Set( false );

            // Start the delay timers
            feedTimer.Start();
            guardTimer.Start();
        }

        /* If frisbee is going to be fed into the shooter and the actuator had
         * enough time to move completely out from underneath the frisbees
         */
        if ( frisbeeFeed.Get() && feedTimer.Get() > 0.3 ) {
            // Reset feed actuator and push frisbee into shooter
            frisbeeFeed.Set( false );

            // Reset timer
            feedTimer.Stop();
            feedTimer.Reset();
        }

        // If frisbee guard is down and frisbee has moved past guard
        if ( frisbeeGuard.Get() && guardTimer.Get() > 0.6 ) {
            frisbeeGuard.Set( true );

            // Reset timer
            guardTimer.Stop();
            guardTimer.Reset();
        }
        /* ========================= */

        /* ===== Control gyro ===== */
        // Reset gyro
        if ( driveStick2Buttons.releasedButton( 8 ) ) {
            testGyro.Reset();
        }

        if ( driveStick2Buttons.releasedButton( 5 ) ) {
            gyroEnabled = true;
        }

        if ( driveStick2Buttons.releasedButton( 6 ) ) {
            gyroEnabled = false;
        }
        /* ======================== */

        // Enable encoders with PID loops
        if ( driveStick2Buttons.releasedButton( 3 ) ) {
            //mainDrive.EnableEncoders( true );
        }

        // Disable encoders with PID loops
        if ( driveStick2Buttons.releasedButton( 4 ) ) {
            //mainDrive.EnableEncoders( false );
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
        if ( gyroEnabled && driveMode != MecanumDrive::Arcade ) {
            joyGyro = testGyro.GetAngle();
        }
        // Compensate for ArcadeDrive robot being backwards
        else if ( driveMode == MecanumDrive::Arcade ) {
            joyGyro = 270.f;
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
        if ( driveMode != MecanumDrive::Arcade ) {
            joyX = driveStick2.GetX();
            joyY = driveStick2.GetY();
        }
        else {
            joyX = driveStick1.GetX();
            joyY = driveStick2.GetY();
        }

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
        else {
            mainDrive.SetDriveMode( driveMode );
        }

        mainDrive.Drive( joyX , joyY , joyTwist , joyGyro );

        Wait( 0.1 );
    }
}
