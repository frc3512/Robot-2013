//=============================================================================
//File Name: OurRobot.cpp
//Description: Main robot class in which all robot sensors and devices are
//             declared
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "OurRobot.hpp"
#include "DriverStationDisplay.hpp"

//#include "SFMLSystem/VxWorks/SleepImpl.hpp"
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

    testGyro( 1 ),

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

    //autonModes.addMethod( "CenterShoot" , &OurRobot::AutonCenter , this );
    autonModes.addMethod( "CenterMove" , &OurRobot::AutonCenterMove , this );
    autonModes.addMethod( "LeftShoot" , &OurRobot::AutonLeftShoot , this );
    autonModes.addMethod( "RightShoot" , &OurRobot::AutonRightShoot , this );

    // Set encoder ports
    mainDrive.SetEncoderPorts( 14 , 13 , 10 , 9 ,
            6 , 5 , 8 , 7 );

    // Let motors run for up to 1 second uncontrolled before shutting them down
    mainDrive.SetExpiration( 1.f );
    mainDrive.SetSafetyEnabled( false );

    mainDrive.SquareInputs( true );

    autonMode = 0;
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

    // floats don't work so " * 100000" saves some precision in a UINT

    driverStation->clear();

    *driverStation << static_cast<std::string>( "display" );

    *driverStation << static_cast<unsigned int>(mainDrive.GetDriveMode());

    *driverStation << static_cast<int>(testGyro.GetAngle() * 1000.f);

    *driverStation << static_cast<bool>( isGyroEnabled );

    *driverStation << static_cast<unsigned int>( ScaleValue(shootStick.GetZ()) * 1000.f );

    *driverStation << static_cast<unsigned int>( frisbeeShooter.getTargetRPM() * 1000.f );

    *driverStation << static_cast<unsigned int>( frisbeeShooter.getRPM() * 1000.f );

    *driverStation << static_cast<bool>( frisbeeShooter.isReady() );

    *driverStation << static_cast<bool>( frisbeeShooter.isShooting() );

    *driverStation << static_cast<bool>( isShooterManual );

    driverStation->sendToDS();

    // Gets messages from DS and fills 'autonMode' if it's a connection message
    const std::string& command = driverStation->receiveFromDS( &autonMode );

    // If the DS just connected, send it a list of available autonomous modes
    if ( std::strcmp( command.c_str() , "connect\r\n" ) == 0 ) {
        driverStation->clear();

        *driverStation << static_cast<std::string>( "autonList" );

        for ( unsigned int i = 0 ; i < autonModes.size() ; i++ ) {
            *driverStation << autonModes.name( i );
        }

        driverStation->sendToDS();
    }
    /* ====================================== */

    DriverStationLCD::GetInstance()->Clear();

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line1 , 1 , "Gyro: %f" , testGyro.GetAngle() );

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line2 , 1 , "Shoot: %f" , frisbeeShooter.getRPM() );
    //DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line2 , 1 , "Shoot: %f" , -ScaleValue(shootStick.GetAxis(Joystick::kZAxis)) );

    if ( mainDrive.GetDriveMode() == MecanumDrive::Omni ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line3 , 1 , "Drive: Omni" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::Strafe ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line3 , 1 , "Drive: Strafe" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::Arcade ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line3 , 1 , "Drive: Arcade" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::FLpivot ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line3 , 1 , "Drive: FLpivot" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::FRpivot ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line3 , 1 , "Drive: FRpivot" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::RLpivot ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line3 , 1 , "Drive: RLpivot" );
    }

    else if ( mainDrive.GetDriveMode() == MecanumDrive::RRpivot ) {
        DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line3 , 1 , "Drive: RRpivot" );
    }
    //DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line3 , 1 , "encFL: %f" , mainDrive.GetFL() );

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line4 , 1 , "ScaleZ: %f" , ScaleValue(shootStick.GetZ()) );
    //DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line4 , 1 , "encRL: %f" , mainDrive.GetRL() );

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line5 , 1 , "encFR: %f" , mainDrive.GetFRrate() );

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line6 , 1 , "encRR: %f" , mainDrive.GetFLrate() );

    DriverStationLCD::GetInstance()->UpdateLCD();
}

START_ROBOT_CLASS(OurRobot);
