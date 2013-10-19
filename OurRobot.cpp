//=============================================================================
//File Name: OurRobot.cpp
//Description: Main robot class in which all robot sensors and devices are
//             declared
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "OurRobot.hpp"

#include <string>
#include <types/vxTypes.h>

float ScaleValue( float value ) {
    // CONSTANT^-1 is step value (now 1/500)
    return floorf( 500.f * ( 1.f - value ) / 2.f ) / 500.f;
}

// Creates a wider band in which the joystick won't make the robot move
double deadband( double value ) {
    if ( fabs( value ) < 0.05 ) {
        return 0;
    }

    return value;
}

OurRobot::OurRobot() :
    Settings( "/ni-rt/system/RobotSettings.txt" ),

    driveStick( 1 ),
    shootStick( 2 ),

    mainCompressor( 1 , 2 ),

    frisbeeFeeder( 1 , 3 , 0.3f , 0.3f ),

    shooterAngle( 2 ),

    flMotor( 3 ),
    rlMotor( 5 ),
    frMotor( 7 ),
    rrMotor( 1 ),
    mainDrive( flMotor , rlMotor , frMotor , rrMotor ,
            14 , 13 , 10 , 9 , 6 , 5 , 8 , 7 ),

    frisbeeShooter( 9 , 10 , 2 , 56 , 4.f ) ,

    climbArms( 4 ),

#ifdef KOP_KGYRO
    fieldGyro( 1 , 1 , 2 , 0xE5 , 0xC4 ) ,
#elif defined NEW_KGYRO
    fieldGyro( 2 , 0x38 , 0xE5 , 0xC4 ) ,
#else
    fieldGyro( 1 ) ,
#endif

    underGlow( 5 ),

    // Field-oriented driving by default
    isGyroEnabled( true ),
    slowRotate( false ),
    isShooterManual( false ),

    // Create a GraphHost
    pidGraph( 3513 )
{
    pidGraph.resetTime();
    pidGraph.setSendInterval( 5 );

    driverStation = DriverStationDisplay<OurRobot>::getInstance( atoi( Settings::getValueFor( "DS_Port" ).c_str() ) );

    driverStation->addAutonMethod( "CenterMove" , &OurRobot::AutonCenterMove , this );
    driverStation->addAutonMethod( "RightMove" , &OurRobot::AutonRightMove , this );
    driverStation->addAutonMethod( "LeftMove" , &OurRobot::AutonLeftMove , this );
    driverStation->addAutonMethod( "TwoDisc" , &OurRobot::AutonTwoDisc , this );

    // Let motors run for up to 1 second uncontrolled before shutting them down
    mainDrive.SetExpiration( 1.f );
    mainDrive.SetDeadband( 0.02f );
    frisbeeShooter.setPID( atof( getValueFor( "PID_SHOOT_P" ).c_str() ) , atof( getValueFor( "PID_SHOOT_I" ).c_str() ) , atof( getValueFor( "PID_SHOOT_D" ).c_str() ) );
    frisbeeShooter.updateEncoderFilter( atof( getValueFor( "SHOOTER_RPM_Q").c_str() ) , atof( getValueFor( "SHOOTER_RPM_R" ).c_str() ) );

    frisbeeShooter.stop();
    mainDrive.SquareInputs( true );

    DSpacketTime.Start();
}

OurRobot::~OurRobot() {

}

void OurRobot::DS_PrintOut() {
    if ( pidGraph.hasIntervalPassed() ) {
        pidGraph.graphData( -mainDrive.GetFRrate() , "FR PID" );
        pidGraph.graphData( mainDrive.GetFRsetpoint() , "FR Setpoint" );
        pidGraph.graphData( frisbeeShooter.getRPM() , "Shoot Filt RPM" );
        pidGraph.graphData( frisbeeShooter.getTargetRPM() , "Shoot Setpoint" );
        pidGraph.graphData( mainDrive.GetFLrate() , "FL PID" );
        pidGraph.graphData( mainDrive.GetFLsetpoint() , "FL Setpoint" );

        pidGraph.resetInterval();
    }

    if ( DSpacketTime.HasPeriodPassed( 0.2 ) ) {
        /* ===== Print to Driver Station LCD =====
         * Packs the following variables:
         *
         * unsigned int: drive mode
         * int: gyro angle
         * bool: isGyroEnabled
         * bool: slowRotate
         * unsigned int: manual RPM
         * unsigned int: target RPM
         * unsigned int: shooter RPM
         * bool: shooterReady
         * bool: isShooting
         * bool: isShooterManual
         */

        driverStation->clear();

        MecanumDrive::DriveMode driveMode = mainDrive.GetDriveMode();
        std::string strDriveMode;
        if ( driveMode == MecanumDrive::Omni ) {
            strDriveMode = "Omni";
        }
        else if ( driveMode == MecanumDrive::Strafe ) {
            strDriveMode = "Strafe";
        }
        else if ( driveMode == MecanumDrive::Arcade ) {
            strDriveMode = "Arcade";
        }
        else if ( driveMode == MecanumDrive::FLpivot ) {
            strDriveMode = "FLpivot";
        }
        else if ( driveMode == MecanumDrive::FRpivot ) {
            strDriveMode = "FRpivot";
        }
        else if ( driveMode == MecanumDrive::RLpivot ) {
            strDriveMode = "RLpivot";
        }
        else if ( driveMode == MecanumDrive::RRpivot ) {
            strDriveMode = "RRpivot";
        }
        DS::AddElementData( driverStation , "MODE" , strDriveMode );

#if defined(KOP_KGYRO) || defined(NEW_KGYRO)
        DS::AddElementData( driverStation , "GYRO_VAL" , static_cast<int32_t>( fieldGyro.getXangle() ) );
//#elif defined NEW_KGYRO
//        DS::AddElementData( driverStation , "GYRO_VAL" , static_cast<int32_t>( fieldGyro.getXangle() ) );
#else
        DS::AddElementData( driverStation , "GYRO_VAL" , static_cast<int32_t>( fieldGyro.GetAngle() ) );
#endif

        if ( isGyroEnabled ) {
            DS::AddElementData( driverStation , "GYRO_ON" , DS::active );
        }
        else {
            DS::AddElementData( driverStation , "GYRO_ON" , DS::inactive );
        }

        if ( slowRotate ) {
            DS::AddElementData( driverStation , "ROTATE" , DS::active );
        }
        else {
            DS::AddElementData( driverStation , "ROTATE" , DS::inactive );
        }

        DS::AddElementData( driverStation , "RPM_MAN_DISP" , 100.f * ScaleValue(shootStick.GetZ()) );
        DS::AddElementData( driverStation , "RPM_MAN" , static_cast<int8_t>( 100.f * ScaleValue(shootStick.GetZ()) ) );

        DS::AddElementData( driverStation , "RPM_SET_DISP" , frisbeeShooter.getTargetRPM() );
        DS::AddElementData( driverStation , "RPM_SET" , static_cast<int8_t>( frisbeeShooter.getTargetRPM() / Shooter::maxSpeed * 100.f ) );

        DS::AddElementData( driverStation , "RPM_REAL_DISP" , frisbeeShooter.getRPM() );
        DS::AddElementData( driverStation , "RPM_REAL" , static_cast<int8_t>( frisbeeShooter.getRPM() / Shooter::maxSpeed * 100.f ) );

        if ( frisbeeShooter.isReady() ) {
            DS::AddElementData( driverStation , "SHOOT_READY" , DS::active );
        }
        else {
            DS::AddElementData( driverStation , "SHOOT_READY" , DS::inactive );
        }

        if ( frisbeeShooter.isShooting() ) {
            DS::AddElementData( driverStation , "SHOOT_ON" , DS::active );
        }
        else {
            DS::AddElementData( driverStation , "SHOOT_ON" , DS::inactive );
        }

        if ( isShooterManual ) {
            DS::AddElementData( driverStation , "SHOOT_MAN" , DS::active );
        }
        else {
            DS::AddElementData( driverStation , "SHOOT_MAN" , DS::inactive );
        }

        if ( !climbArms.Get() ) {
            DS::AddElementData( driverStation , "ARMS_DOWN" , DS::active );
        }
        else {
            DS::AddElementData( driverStation , "ARMS_DOWN" , DS::inactive );
        }

        driverStation->sendToDS();
    }

    driverStation->receiveFromDS();
    /* ====================================== */
}

START_ROBOT_CLASS(OurRobot);
