//=============================================================================
//File Name: OurRobot.cpp
//Description: Main robot class in which all robot sensors and devices are
//             declared
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "OurRobot.hpp"
#include "DriverStationDisplay.hpp"

float ScaleZ( Joystick& stick) {
    // CONSTANT^-1 is step value (now 1/500)
    return floorf( 500.f * ( 1.f - stick.GetZ() ) / 2.f ) / 500.f;
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

    mainCompressor( 1 , 6 ),
    flMotor( 3 ),
    rlMotor( 5 ),
    frMotor( 1 ),
    rrMotor( 6 ),
    mainDrive( flMotor , rlMotor , frMotor , rrMotor ),
    driveStick1( 1 ),
    driveStick2( 2 ),
    cameraStick( 3 ),

    shooterMotor1( 7 ),
    shooterMotor2( 8 ),

    // single board computer's IP address and port
    turretKinect( getValueFor( "SBC_IP" ) , atoi( getValueFor( "SBC_Port" ).c_str() ) ),
    testGyro( 1 )
    //camYTilt( 10 ),
    //camXTilt( 9 )
{
    driverStation = DriverStationDisplay::getInstance( atoi( Settings::getValueFor( "DS_Port" ).c_str() ) );

    autonModes.addMethod( "Shoot" , &OurRobot::AutonShoot , this );
    autonModes.addMethod( "Feed" , &OurRobot::AutonFeed , this );

    // Set encoder ports
    mainDrive.SetEncoderPorts( 14 , 13 , 10 , 9 ,
            6 , 5 , 8 , 7 );

    // Let motors run for up to 1 second uncontrolled before shutting them down
    mainDrive.SetExpiration( 1.f );

    mainDrive.SquareInputs( true );

    autonMode = 0;
}

OurRobot::~OurRobot() {
    DriverStationDisplay::freeInstance();
    driverStation = NULL;
}

void OurRobot::DS_PrintOut() {
    /* ===== Print to Driver Station LCD =====
     * Packs the following variables:
     *
     * std::string: type of data (either "display" or "autonList")
     * unsigned int: drive1 ScaleZ
     * unsigned int: drive2 ScaleZ
     * unsigned int: turret ScaleZ
     * bool: drivetrain is in low gear
     * unsigned char: is hammer mechanism deployed
     * unsigned int: shooter RPM
     * bool: shooter RPM control is manual
     * bool: isShooting
     * bool: isAutoAiming
     * bool: turret is locked on
     * unsigned char: Kinect is online
     * unsigned int: distance to target
     */

    // floats don't work so " * 100000" saves some precision in a UINT

    driverStation->clear();

    *driverStation << static_cast<std::string>( "display" );

    *driverStation << static_cast<unsigned int>(ScaleZ(driveStick1) * 100000.f);

    *driverStation << static_cast<unsigned int>(ScaleZ(driveStick2) * 100000.f);

    *driverStation << static_cast<unsigned int>(ScaleZ(cameraStick) * 100000.f);

    *driverStation << static_cast<bool>( fabs( turretKinect.getPixelOffset() ) < TurretKinect::pxlDeadband
            && turretKinect.getOnlineStatus() == sf::Socket::Done );

    *driverStation << static_cast<unsigned char>( turretKinect.getOnlineStatus() );

    *driverStation << turretKinect.getDistance();

    driverStation->sendToDS();

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
    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line1 , 1 , "Gyro: %.3f" , testGyro.GetAngle() );
    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line2 , 1 , "Joy: %.3f, %.3f, %.3f" , driveStick2.GetX() , driveStick2.GetY() , driveStick2.GetZ() );
    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line3 , 1 , "encFL: %f" , mainDrive.GetFL() );
    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line4 , 1 , "encRL: %f" , mainDrive.GetRL() );
    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line5 , 1 , "encFR: %f" , mainDrive.GetFR() );
    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line6 , 1 , "encRR: %f" , mainDrive.GetRR() );
    DriverStationLCD::GetInstance()->UpdateLCD();
}

START_ROBOT_CLASS(OurRobot);
