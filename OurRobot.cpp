//=============================================================================
//File Name: OurRobot.cpp
//Description: Main robot class in which all robot sensors and devices are
//             declared
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "OurRobot.hpp"
#include "DriverStationDisplay.hpp"

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cstring>
#include <stdint.h>

#include <fstream>
#include <sstream>
#include <string>

/* Declare typedefs for wide C++ streams since the VxWorks 6.3 headers usually
 * fail to define them
 */
namespace std {
typedef basic_stringstream<char32_t, char_traits<char32_t>,
    allocator<char32_t> > w32stringstream;

typedef std::basic_ifstream<char32_t, std::char_traits<char32_t> > w32ifstream;
}

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
    mainDrive( flMotor , rlMotor , frMotor , rrMotor ),

    frisbeeShooter( 9 , 10 , 2 , 56 , 4.f ) ,

    climbArms( 4 ),

#ifdef NEW_GYRO
    fieldGyro( 1 , 1 , 0x38 , 0xE5 ) ,
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

    driverStation = DriverStationDisplay::getInstance( atoi( Settings::getValueFor( "DS_Port" ).c_str() ) );

    autonModes.addMethod( "CenterMove" , &OurRobot::AutonCenterMove , this );
    autonModes.addMethod( "RightMove" , &OurRobot::AutonRightMove , this );
    autonModes.addMethod( "LeftMove" , &OurRobot::AutonLeftMove , this );
    autonModes.addMethod( "TwoDisc" , &OurRobot::AutonTwoDisc , this );

    // Set encoder ports
    mainDrive.SetEncoderPorts( 14 , 13 , 10 , 9 ,
            6 , 5 , 8 , 7 );

    // Let motors run for up to 1 second uncontrolled before shutting them down
    mainDrive.SetExpiration( 1.f );
    frisbeeShooter.setPID( atof( getValueFor( "PID_CLOSE_P" ).c_str() ) , atof( getValueFor( "PID_CLOSE_I" ).c_str() ) , atof( getValueFor( "PID_CLOSE_D" ).c_str() ) );

#if 0
    PIDConst constants;

    constants.P = 0.f;
    constants.I = 0.f;
    constants.D = 0.f;
    constants.F = 1.f / Shooter::maxSpeed;
    constants.setpoint = 0.f;
    frisbeeShooter.addPIDConst( constants );

    constants.P = atof( getValueFor( "PID_CLOSE_P" ).c_str() );
    constants.I = atof( getValueFor( "PID_CLOSE_I" ).c_str() );
    constants.D = atof( getValueFor( "PID_CLOSE_D" ).c_str() );
    constants.F = 0.f;
    constants.setpoint = 0.f;
    frisbeeShooter.addPIDConst( constants );
#endif

    frisbeeShooter.stop();
    mainDrive.SquareInputs( true );

    autonMode = 3;

    DSpacketTime.Start();
}

OurRobot::~OurRobot() {

}

void OurRobot::DS_PrintOut() {
    if ( pidGraph.hasIntervalPassed() ) {
        //pidGraph.graphData( 5000.f , "PID0" );
        pidGraph.graphData( frisbeeShooter.getRPM() , "PID0" );
        pidGraph.graphData( frisbeeShooter.getTargetRPM() , "PID1" );

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

        *driverStation << static_cast<std::string>( "display\r\n" );

        unsigned int driveMode = mainDrive.GetDriveMode();
        std::w32string strDriveMode;
        if ( driveMode == MecanumDrive::Omni ) {
            strDriveMode = U"Omni";
        }
        else if ( driveMode == MecanumDrive::Strafe ) {
            strDriveMode = U"Strafe";
        }
        else if ( driveMode == MecanumDrive::Arcade ) {
            strDriveMode = U"Arcade";
        }
        else if ( driveMode == MecanumDrive::FLpivot ) {
            strDriveMode = U"FLpivot";
        }
        else if ( driveMode == MecanumDrive::FRpivot ) {
            strDriveMode = U"FRpivot";
        }
        else if ( driveMode == MecanumDrive::RLpivot ) {
            strDriveMode = U"RLpivot";
        }
        else if ( driveMode == MecanumDrive::RRpivot ) {
            strDriveMode = U"RRpivot";
        }
        driverStation->addElementData( 's' , U"MODE" , strDriveMode );

#ifdef NEW_GYRO
        driverStation->addElementData( 'i' , U"GYRO_VAL" , static_cast<int32_t>( fieldGyro.getXangle() ) );
#else
        driverStation->addElementData( 'i' , U"GYRO_VAL" , static_cast<int32_t>( fieldGyro.GetAngle() ) );
#endif

        if ( isGyroEnabled ) {
            driverStation->addElementData( 'c' , U"GYRO_ON" , static_cast<uint8_t>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , U"GYRO_ON" , static_cast<uint8_t>( 2 ) );
        }

        if ( slowRotate ) {
            driverStation->addElementData( 'c' , U"ROTATE" , static_cast<uint8_t>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , U"ROTATE" , static_cast<uint8_t>( 2 ) );
        }

        {
        std::w32stringstream ss;
        ss << ScaleValue(shootStick.GetZ());
        driverStation->addElementData( 's' , U"RPM_MAN_DISP" , ss.str() );
        wprintf( "RPM_MAN_DISP=%ls\n" , ss.str() );
        }

        driverStation->addElementData( 'c' , U"RPM_MAN" , static_cast<unsigned char>( ScaleValue(shootStick.GetZ()) * 100.f ) );

        {
        std::w32stringstream ss;
        ss << frisbeeShooter.getTargetRPM();
        driverStation->addElementData( 's' , U"RPM_SET_DISP" , ss.str() );
        }

        driverStation->addElementData( 'c' , U"RPM_SET" , static_cast<unsigned char>( frisbeeShooter.getTargetRPM() / Shooter::maxSpeed * 100.f ) );

        {
        std::w32stringstream ss;
        ss << frisbeeShooter.getRPM();
        driverStation->addElementData( 's' , U"RPM_REAL_DISP" , ss.str() );
        }

        driverStation->addElementData( 'c' , U"RPM_REAL" , static_cast<unsigned char>( frisbeeShooter.getRPM() / Shooter::maxSpeed * 100.f ) );

        if ( frisbeeShooter.isReady() ) {
            driverStation->addElementData( 'c' , U"SHOOT_READY" , static_cast<unsigned char>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , U"SHOOT_READY" , static_cast<unsigned char>( 2 ) );
        }

        if ( frisbeeShooter.isShooting() ) {
            driverStation->addElementData( 'c' , U"SHOOT_ON" , static_cast<unsigned char>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , U"SHOOT_ON" , static_cast<unsigned char>( 2 ) );
        }

        if ( isShooterManual ) {
            driverStation->addElementData( 'c' , U"SHOOT_MAN" , static_cast<unsigned char>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , U"SHOOT_MAN" , static_cast<unsigned char>( 2 ) );
        }

        if ( !climbArms.Get() ) {
            driverStation->addElementData( 'c' , U"ARMS_DOWN" , static_cast<unsigned char>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , U"ARMS_DOWN" , static_cast<unsigned char>( 2 ) );
        }

        driverStation->sendToDS();
    }

    // Gets messages from DS and fills 'autonMode' if it's a connection message
    const std::string& command = driverStation->receiveFromDS( &autonMode );

    if ( std::strcmp( command.c_str() , "connect\r\n" ) == 0 ) {
        // Send GUI element file to DS
        driverStation->clear();

        *driverStation << static_cast<std::string>( "guiCreate\r\n" );

        FILE *fp;
        unsigned char *tmpbuf;
        size_t bytesread;
        uint32_t filesize;

        // Open the file
        fp = std::fopen("/ni-rt/system/GUISettings.txt", "rb");

        if( fp != NULL ) {
            // Get its length
            std::fseek( fp , 0 , SEEK_END );
            filesize = std::ftell( fp );
            filesize++;
            std::fseek( fp , 0 , SEEK_SET );

            // Send the length
            *driverStation << filesize;

            // Allocate a buffer for the file
            tmpbuf = static_cast<unsigned char*>(std::malloc(filesize));

            // Send the data TODO: htonl() the data before it's sent
            bytesread = std::fread( tmpbuf , 1 , filesize , fp );
            driverStation->append( tmpbuf , bytesread );

            std::fclose( fp );
            std::free( tmpbuf );
        }

        driverStation->sendToDS();

        // Send a list of available autonomous modes
        driverStation->clear();

        *driverStation << static_cast<std::string>( "autonList\r\n" );

        for ( unsigned int i = 0 ; i < autonModes.size() ; i++ ) {
            *driverStation << autonModes.name( i );
        }

        driverStation->sendToDS();

        // Make sure driver knows which autonomous mode is selected
        driverStation->clear();

        *driverStation << static_cast<std::string>( "autonConfirmed\r\n" );
        *driverStation << autonModes.name( autonMode );

        driverStation->sendToDS();
    }
    else if ( std::strcmp( command.c_str() , "autonSelect\r\n" ) == 0 ) {
        driverStation->clear();

        *driverStation << static_cast<std::string>( "autonConfirmed\r\n" );
        *driverStation << autonModes.name( autonMode );

        driverStation->sendToDS();
    }
    /* ====================================== */
}

START_ROBOT_CLASS(OurRobot);
