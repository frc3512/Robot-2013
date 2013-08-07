//=============================================================================
//File Name: OurRobot.cpp
//Description: Main robot class in which all robot sensors and devices are
//             declared
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "OurRobot.hpp"
#include "DriverStationDisplay.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <cstring>
#include <types/vxTypes.h>

/* Declare typedef for wide C++ string since the VxWorks 6.3 headers usually
 * fail to define it
 */
namespace std {
typedef basic_string<wchar_t> wstring;
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

    driverStation = DriverStationDisplay::getInstance( atoi( Settings::getValueFor( "DS_Port" ).c_str() ) );

    autonModes.addMethod( "CenterMove" , &OurRobot::AutonCenterMove , this );
    autonModes.addMethod( "RightMove" , &OurRobot::AutonRightMove , this );
    autonModes.addMethod( "LeftMove" , &OurRobot::AutonLeftMove , this );
    autonModes.addMethod( "TwoDisc" , &OurRobot::AutonTwoDisc , this );

    // Retrieve stored autonomous index
    std::ifstream autonModeFile( "autonMode.txt" );
    if ( autonModeFile.is_open() ) {
        autonModeFile >> autonMode;

        autonModeFile.close();
    }
    else {
        autonMode = 0;
    }

    // Let motors run for up to 1 second uncontrolled before shutting them down
    // TODO Enable safety after testing
    //mainDrive.SetExpiration( 1.f );
    mainDrive.SetSafetyEnabled( false );
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

    DSpacketTime.Start();
}

OurRobot::~OurRobot() {

}

void OurRobot::DS_PrintOut() {
    if ( pidGraph.hasIntervalPassed() ) {
        pidGraph.graphData( mainDrive.GetFLrate() , "FL PID" );
        pidGraph.graphData( mainDrive.GetFLsetpoint() , "FL Setpoint" );
        pidGraph.graphData( -mainDrive.GetFRrate() , "FR PID" );
        pidGraph.graphData( mainDrive.GetFRsetpoint() , "FR Setpoint" );
        pidGraph.graphData( mainDrive.GetRLrate() , "RL PID" );
        pidGraph.graphData( mainDrive.GetRLsetpoint() , "RL Setpoint" );
        pidGraph.graphData( -mainDrive.GetRRrate() , "RR PID" );
        pidGraph.graphData( mainDrive.GetRRsetpoint() , "RR Setpoint" );

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
        std::wstring strDriveMode;
        if ( driveMode == MecanumDrive::Omni ) {
            strDriveMode = L"Omni";
        }
        else if ( driveMode == MecanumDrive::Strafe ) {
            strDriveMode = L"Strafe";
        }
        else if ( driveMode == MecanumDrive::Arcade ) {
            strDriveMode = L"Arcade";
        }
        else if ( driveMode == MecanumDrive::FLpivot ) {
            strDriveMode = L"FLpivot";
        }
        else if ( driveMode == MecanumDrive::FRpivot ) {
            strDriveMode = L"FRpivot";
        }
        else if ( driveMode == MecanumDrive::RLpivot ) {
            strDriveMode = L"RLpivot";
        }
        else if ( driveMode == MecanumDrive::RRpivot ) {
            strDriveMode = L"RRpivot";
        }
        driverStation->addElementData( 's' , L"MODE" , strDriveMode );

#if defined(KOP_KGYRO) || defined(NEW_KGYRO)
        driverStation->addElementData( 'i' , L"GYRO_VAL" , static_cast<int32_t>( fieldGyro.getXangle() ) );
//#elif defined NEW_KGYRO
//        driverStation->addElementData( 'i' , L"GYRO_VAL" , static_cast<int32_t>( fieldGyro.getXangle() ) );
#else
        driverStation->addElementData( 'i' , L"GYRO_VAL" , static_cast<int32_t>( fieldGyro.GetAngle() ) );
#endif

        if ( isGyroEnabled ) {
            driverStation->addElementData( 'c' , L"GYRO_ON" , static_cast<uint8_t>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , L"GYRO_ON" , static_cast<uint8_t>( 2 ) );
        }

        if ( slowRotate ) {
            driverStation->addElementData( 'c' , L"ROTATE" , static_cast<uint8_t>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , L"ROTATE" , static_cast<uint8_t>( 2 ) );
        }

        {
        std::stringstream ss;
        ss << 100.f * ScaleValue(shootStick.GetZ());

        driverStation->addElementData( 's' , L"RPM_MAN_DISP" , ss.str() );
        }

        driverStation->addElementData( 'c' , L"RPM_MAN" , static_cast<unsigned char>( ScaleValue(shootStick.GetZ()) * 100.f ) );

        {
        std::stringstream ss;
        ss << frisbeeShooter.getTargetRPM();

        driverStation->addElementData( 's' , L"RPM_SET_DISP" , ss.str() );
        }

        driverStation->addElementData( 'c' , L"RPM_SET" , static_cast<unsigned char>( frisbeeShooter.getTargetRPM() / Shooter::maxSpeed * 100.f ) );

        {
        std::stringstream ss;
        ss << frisbeeShooter.getRPM();

        driverStation->addElementData( 's' , L"RPM_REAL_DISP" , ss.str() );
        }

        driverStation->addElementData( 'c' , L"RPM_REAL" , static_cast<unsigned char>( frisbeeShooter.getRPM() / Shooter::maxSpeed * 100.f ) );

        if ( frisbeeShooter.isReady() ) {
            driverStation->addElementData( 'c' , L"SHOOT_READY" , static_cast<unsigned char>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , L"SHOOT_READY" , static_cast<unsigned char>( 2 ) );
        }

        if ( frisbeeShooter.isShooting() ) {
            driverStation->addElementData( 'c' , L"SHOOT_ON" , static_cast<unsigned char>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , L"SHOOT_ON" , static_cast<unsigned char>( 2 ) );
        }

        if ( isShooterManual ) {
            driverStation->addElementData( 'c' , L"SHOOT_MAN" , static_cast<unsigned char>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , L"SHOOT_MAN" , static_cast<unsigned char>( 2 ) );
        }

        if ( !climbArms.Get() ) {
            driverStation->addElementData( 'c' , L"ARMS_DOWN" , static_cast<unsigned char>( 0 ) );
        }
        else {
            driverStation->addElementData( 'c' , L"ARMS_DOWN" , static_cast<unsigned char>( 2 ) );
        }

        driverStation->sendToDS();
    }

    // Gets messages from DS and fills 'autonMode' if it's a connection message
    const std::string& command = driverStation->receiveFromDS( &autonMode );

    if ( std::strcmp( command.c_str() , "connect\r\n" ) == 0 ) {
        // Send GUI element file to DS
        driverStation->clear();

        *driverStation << static_cast<std::string>( "guiCreate\r\n" );

        // Open the file
        std::ifstream guiFile(
                "/ni-rt/system/GUISettings.txt" , std::ifstream::binary );

        if( guiFile.is_open() ) {
            // Get its length
            guiFile.seekg( 0 , guiFile.end );
            unsigned int fileSize = guiFile.tellg();
            guiFile.seekg( 0 , guiFile.beg );

            // Send the length
            *driverStation << static_cast<uint32_t>(fileSize);

            // Allocate a buffer for the file
            char* tempBuf = new char[fileSize];

            // Send the data TODO: htonl() the data before it's sent
            guiFile.read( tempBuf , fileSize );
            driverStation->append( tempBuf , fileSize );

            delete[] tempBuf;
            guiFile.close();
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

        // Store newest autonomous choice to file for persistent storage
        std::ofstream autonModeFile( "autonMode.txt" , std::ofstream::trunc );
        if ( autonModeFile.is_open() ) {
            autonModeFile << autonMode;

            autonModeFile.close();
        }

        driverStation->sendToDS();
    }
    /* ====================================== */
}

START_ROBOT_CLASS(OurRobot);
