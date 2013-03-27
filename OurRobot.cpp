//=============================================================================
//File Name: OurRobot.cpp
//Description: Main robot class in which all robot sensors and devices are
//             declared
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "OurRobot.hpp"
#include "DriverStationDisplay.hpp"

#include <fstream>
#include <sstream>

namespace std {
typedef basic_stringstream<wchar_t, char_traits<wchar_t>,
    allocator<wchar_t> > wstringstream;

typedef std::basic_ifstream<wchar_t, std::char_traits<wchar_t> > wifstream;
}

#include <string>

int gettimeofday (struct timeval *tv_ptr, void *ptr);

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

    frisbeeFeeder( 1 , 3 , 0.3f , 0.65f ),

    shooterAngle( 2 ),

    flMotor( 3 ),
    rlMotor( 5 ),
    frMotor( 7 ),
    rrMotor( 1 ),
    mainDrive( flMotor , rlMotor , frMotor , rrMotor ),

    frisbeeShooter( 9 , 10 , 2 , 56 , 4.f ) ,

    leftClimbArm( 4 ),
    rightClimbArm( 3 ),

    fieldGyro( 1 ),

    underGlow( 5 ),

    // Field-oriented driving by default
    isGyroEnabled( true ),
    slowRotate( false ),
    isShooterManual( false ),

    // Create a GraphHost
    pidGraph( 3513 )
{
    struct timeval rawTime;

    /* Store the current time into startTime as the fixed starting point
     * for our graph.
     */
    gettimeofday( &rawTime , NULL );
    startTime = rawTime.tv_usec / 1000 + rawTime.tv_sec * 1000;
    lastTime = startTime;

    driverStation = DriverStationDisplay::getInstance( atoi( Settings::getValueFor( "DS_Port" ).c_str() ) );

    autonModes.addMethod( "CenterMove" , &OurRobot::AutonCenterMove , this );
    autonModes.addMethod( "LeftMove" , &OurRobot::AutonLeftMove , this );
    autonModes.addMethod( "TwoDisc" , &OurRobot::AutonTwoDisc , this );
    //autonModes.addMethod( "CenterShoot" , &OurRobot::AutonCenterShoot , this );

    // Set encoder ports
    mainDrive.SetEncoderPorts( 14 , 13 , 10 , 9 ,
            6 , 5 , 8 , 7 );

    // Let motors run for up to 1 second uncontrolled before shutting them down
    mainDrive.SetExpiration( 1.f );
    frisbeeShooter.setPID( atof( getValueFor( "PID_P" ).c_str() ) , atof( getValueFor( "PID_I" ).c_str() ) , atof( getValueFor( "PID_D" ).c_str() ) );
    frisbeeShooter.stop();
    mainDrive.SquareInputs( true );

    autonMode = 2;
}

OurRobot::~OurRobot() {

}

void OurRobot::DS_PrintOut() {
    struct timeval rawTime;
    uint32_t currentTime;

    gettimeofday( &rawTime , NULL );
    currentTime = rawTime.tv_usec / 1000 + rawTime.tv_sec * 1000;

    if ( currentTime - lastTime > 5 ) {
        //pidGraph.graphData( currentTime - startTime , 5000.f , "PID0" );
        pidGraph.graphData( currentTime - startTime , frisbeeShooter.getRPM() , "PID0" );
        pidGraph.graphData( currentTime - startTime , frisbeeShooter.getTargetRPM() , "PID1" );

        lastTime = currentTime;
    }

#if 1
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

    *driverStation << static_cast<std::string>( "display" );

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

    driverStation->addElementData( 'i' , L"GYRO_VAL" , static_cast<int>( fieldGyro.GetAngle() ) );

    driverStation->addElementData( 'c' , L"GYRO_ON" , static_cast<unsigned char>( isGyroEnabled ) );

    driverStation->addElementData( 'c' , L"ROTATE" , static_cast<unsigned char>( slowRotate ) );

    {
    std::wstringstream ss;
    ss << ScaleValue(shootStick.GetZ());
    driverStation->addElementData( 's' , L"RPM_MAN_DISP" , ss.str() );
    }

    driverStation->addElementData( 'c' , L"RPM_MAN" , static_cast<unsigned char>( ScaleValue(shootStick.GetZ()) * 100.f ) );

    {
    std::wstringstream ss;
    ss << frisbeeShooter.getTargetRPM();
    driverStation->addElementData( 's' , L"RPM_SET_DISP" , ss.str() );
    }

    driverStation->addElementData( 'c' , L"RPM_SET" , static_cast<unsigned char>( frisbeeShooter.getTargetRPM() / Shooter::maxSpeed * 100.f ) );

    {
    std::wstringstream ss;
    ss << frisbeeShooter.getRPM();
    driverStation->addElementData( 's' , L"RPM_REAL_DISP" , ss.str() );
    }

    driverStation->addElementData( 'c' , L"RPM_REAL" , static_cast<unsigned char>( frisbeeShooter.getRPM() / Shooter::maxSpeed * 100.f ) );

    driverStation->addElementData( 'c' , L"SHOOT_READY" , static_cast<unsigned char>( frisbeeShooter.isReady() ) );

    driverStation->addElementData( 'c' , L"SHOOT_ON" , static_cast<unsigned char>( frisbeeShooter.isShooting() ) );

    driverStation->addElementData( 'c' , L"SHOOT_MAN" , static_cast<unsigned char>( isShooterManual ) );

    driverStation->sendToDS();
#endif

    // Gets messages from DS and fills 'autonMode' if it's a connection message
    const std::string& command = driverStation->receiveFromDS( &autonMode );

    if ( std::strcmp( command.c_str() , "connect\r\n" ) == 0 ) {
        // Send GUI element file to DS
        driverStation->clear();

        *driverStation << static_cast<std::string>( "guiCreate" );

        std::wifstream guiFile( "GUISettings.txt" );
        std::wstring guiString;

        if ( guiFile.is_open() ) {
            while ( !guiFile.eof() ) {
                std::getline( guiFile , guiString );
                *driverStation << guiString;
            }
        }

        driverStation->sendToDS();

        // Send a list of available autonomous modes
        driverStation->clear();

        *driverStation << static_cast<std::string>( "autonList" );

        for ( unsigned int i = 0 ; i < autonModes.size() ; i++ ) {
            *driverStation << autonModes.name( i );
        }

        driverStation->sendToDS();
    }
    else if ( std::strcmp( command.c_str() , "autonSelect\r\n" ) == 0 ) {
        driverStation->clear();

        *driverStation << static_cast<std::string>( "autonConfirmed" );
        *driverStation << autonModes.name( autonMode );

        driverStation->sendToDS();
    }
    /* ====================================== */

    DriverStationLCD::GetInstance()->Clear();

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line1 , 1 , "Gyro: %f" , fieldGyro.GetAngle() );

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line2 , 1 , "RPM: %f" , frisbeeShooter.getRPM() );

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line3 , 1 , "Target: %f" , frisbeeShooter.getTargetRPM() );

    if ( mainDrive.GetDriveMode() == MecanumDrive::Omni ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line4 , 1 , "Drive: Omni" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::Strafe ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line4 , 1 , "Drive: Strafe" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::Arcade ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line4 , 1 , "Drive: Arcade" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::FLpivot ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line4 , 1 , "Drive: FLpivot" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::FRpivot ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line4 , 1 , "Drive: FRpivot" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::RLpivot ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line4 , 1 , "Drive: RLpivot" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::RRpivot ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line4 , 1 , "Drive: RRpivot" );
    }

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line5 , 1 , "ScaleZ: %f" , ScaleValue(shootStick.GetZ()) );

    if ( frisbeeShooter.isShooting() ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line6 , 1 , "Shooter ON" );
    }
    else {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line6 , 1 , "Shooter OFF" );
    }

    if ( frisbeeShooter.isReady() ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line6 , 13 , "READY" );
    }

    DriverStationLCD::GetInstance()->UpdateLCD();
}

START_ROBOT_CLASS(OurRobot);
