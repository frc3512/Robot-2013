//=============================================================================
//File Name: OurRobot.cpp
//Description: Main robot class in which all robot sensors and devices are
//             declared
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "OurRobot.hpp"
#include "DriverStationDisplay.hpp"

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

    driveStick1( 1 ),
    driveStick2( 2 ),
    shootStick( 3 ),

    mainCompressor( 1 , 2 ),

    frisbeeFeed( 1 ),
    frisbeeGuard( 3 ),

    shooterAngle( 2 ),

    shooterEncoder( 2 , 56 , 4.f ),

    flMotor( 3 ),
    rlMotor( 5 ),
    frMotor( 6 ),
    rrMotor( 1 ),
    mainDrive( flMotor , rlMotor , frMotor , rrMotor ),

    shooterMotor1( 9 ),
    shooterMotor2( 10 ),
    leftClimbArm( 4 ),
    rightClimbArm( 3 ),

    // single board computer's IP address and port
    turretKinect( getValueFor( "SBC_IP" ) , atoi( getValueFor( "SBC_Port" ).c_str() ) ),
    testGyro( 1 )
    //camYTilt( 10 ),
    //camXTilt( 2 )
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

    *driverStation << static_cast<unsigned int>(ScaleValue(driveStick1.GetZ()) * 100000.f);

    *driverStation << static_cast<unsigned int>(ScaleValue(driveStick2.GetZ()) * 100000.f);

    *driverStation << static_cast<unsigned int>(ScaleValue(shootStick.GetZ()) * 100000.f);

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

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line1 , 1 , "Gyro: %f" , testGyro.GetAngle() );

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line2 , 1 , "Shoot: %f" , shooterEncoder.getRPM() );
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

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line5 , 1 , "encFR: %f" , mainDrive.GetFR() );

    DriverStationLCD::GetInstance()->Printf( DriverStationLCD::kUser_Line6 , 1 , "encRR: %f" , mainDrive.GetRR() );

    DriverStationLCD::GetInstance()->UpdateLCD();
}

START_ROBOT_CLASS(OurRobot);
