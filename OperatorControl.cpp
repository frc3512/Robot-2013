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
    frisbeeShooter.setControlMode( Shooter::PID );

    // Turn on blue underglow
    underGlow.Set( Relay::kOn );
    underGlow.Set( Relay::kForward );

    ButtonTracker driveStickButtons( 1 );
    ButtonTracker shootStickButtons( 2 );

    float joyX = 0.f;
    float joyY = 0.f;
    float joyTwist = 0.f;
    float joyGyro = 0.f;

    // Set initial drive mode
    MecanumDrive::DriveMode driveMode = MecanumDrive::Omni;
    mainDrive.SetDriveMode( driveMode );

    // TODO Enable encoders
    //mainDrive.EnableEncoders( true );

    while ( IsEnabled() && IsOperatorControl() ) {
        DS_PrintOut();

        // update "new" value of joystick buttons
        driveStickButtons.updateButtons();
        shootStickButtons.updateButtons();

        /* ============== Toggle Shooter Motors ============== */
        // turns shooter on/off
        if ( shootStickButtons.releasedButton( 4 ) ) {
            frisbeeShooter.start();
        }
        else if ( shootStickButtons.releasedButton( 5 ) ) {
            frisbeeShooter.stop();
        }

        if ( frisbeeShooter.isShooting() ) {
            frisbeeShooter.setScale( ScaleValue( shootStick.GetZ() ) );
        }
        else {
            frisbeeShooter.stop();
        }

        // Toggle manual RPM setting vs setting with encoder input
        if ( shootStickButtons.releasedButton( 8 ) ) {
            isShooterManual = false;

            // Reset PID constants
            Settings::update();

            mainDrive.ReloadPID();

            frisbeeShooter.setPID( atof( getValueFor( "PID_SHOOT_P" ).c_str() ) , atof( getValueFor( "PID_SHOOT_I" ).c_str() ) , atof( getValueFor( "PID_SHOOT_D" ).c_str() ) );
            frisbeeShooter.updateEncoderFilter( atof( getValueFor( "SHOOTER_RPM_Q").c_str() ) , atof( getValueFor( "SHOOTER_RPM_R" ).c_str() ) );

            frisbeeShooter.setControlMode( Shooter::PID );
        }

        if ( shootStickButtons.releasedButton( 9 ) ) {
            isShooterManual = true;

            frisbeeShooter.setControlMode( Shooter::Manual );
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
        /* Don't let a frisbee into the shooter if the shooter wheel isn't
         * spinning, except in the case of a manual override.
         */
        if ( (frisbeeShooter.getRPM() > 500 && !isShooterManual) || isShooterManual ) {
            if ( shootStickButtons.releasedButton( 1 ) ) {
                frisbeeFeeder.activate();
            }
        }

        // Updates state of feed actuators
        frisbeeFeeder.update();
        /* ========================= */

        /* ===== Control climbing mechanism ===== */
        // Arms up
        if ( shootStickButtons.releasedButton( 6 ) ) {
            climbArms.Set( true );
        }

        // Arms down
        if ( shootStickButtons.releasedButton( 7 ) ) {
            climbArms.Set( false );
        }
        /* ====================================== */

        /* ===== Control gyro ===== */
        // Reset gyro
        if ( driveStickButtons.releasedButton( 8 ) ) {
#if defined(KOP_KGYRO) || defined(NEW_KGYRO)
            fieldGyro.resetXangle( 180.f );
#else
            fieldGyro.Reset();
#endif
        }

        if ( driveStickButtons.releasedButton( 5 ) ) {
            isGyroEnabled = true;

            // kForward turns on blue lights
            underGlow.Set( Relay::kOn );
            underGlow.Set( Relay::kForward );
        }

        if ( driveStickButtons.releasedButton( 6 ) ) {
            isGyroEnabled = false;

            // kReverse turns on red lights
            underGlow.Set( Relay::kOn );
            underGlow.Set( Relay::kReverse );
        }
        /* ======================== */

        // Enable encoders with PID loops
        if ( driveStickButtons.releasedButton( 3 ) ) {
            mainDrive.EnableEncoders( true );
        }

        // Disable encoders with PID loops
        if ( driveStickButtons.releasedButton( 4 ) ) {
            mainDrive.EnableEncoders( false );
        }

        // Compensate with gyro angle if that's enabled
        if ( isGyroEnabled && driveMode != MecanumDrive::Arcade ) {
#if defined(KOP_KGYRO) || defined(NEW_KGYRO)
            joyGyro = fieldGyro.getXangle();
#else
            joyGyro = fieldGyro.GetAngle();
#endif
        }
        // Compensate for ArcadeDrive robot being backwards
        else if ( driveMode == MecanumDrive::Arcade ) {
            joyGyro = 90.f;
        }
        else {
            joyGyro = 0.f;
        }

        // While the thumb button is pressed, don't allow rotation
        if ( driveStick.GetRawButton( 2 ) ) {
            joyTwist = 0.f;
        }
        else {
            joyTwist = driveStick.GetZ();
        }

        joyX = driveStick.GetX();
        joyY = driveStick.GetY();

        // Cycle through driving modes
        if ( driveStickButtons.releasedButton( 7 ) ) {
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
        if ( driveStick.GetRawButton( 9 ) ) { // FL wheel pivot
            mainDrive.SetDriveMode( MecanumDrive::FLpivot );
        }
        else if ( driveStick.GetRawButton( 10 ) ) { // FR wheel pivot
            mainDrive.SetDriveMode( MecanumDrive::FRpivot );
        }
        else if ( driveStick.GetRawButton( 11 ) ) { // RL wheel pivot
            mainDrive.SetDriveMode( MecanumDrive::RLpivot );
        }
        else if ( driveStick.GetRawButton( 12 ) ) { // RR wheel pivot
            mainDrive.SetDriveMode( MecanumDrive::RRpivot );
        }
        else {
            mainDrive.SetDriveMode( driveMode );
        }

        // If in lower half, go half speed
        if ( ScaleValue( driveStick.GetTwist() ) < 0.5 ) {
            slowRotate = true;
            joyTwist /= 2.f;
        }
        else {
            slowRotate = false;
        }
        // ^ Goes normal speed if in higher half

        mainDrive.Drive( joyX , joyY , joyTwist , joyGyro );

        Wait( 0.05 );
    }
}
