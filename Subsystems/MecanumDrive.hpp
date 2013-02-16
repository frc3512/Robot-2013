//=============================================================================
//File Name: MecanumDrive.hpp
//Description: Drives the robot with Mecanum wheels in Tank drive configuration
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef MECANUM_DRIVE_HPP
#define MECANUM_DRIVE_HPP

#include "RobotDrive.h"

class Encoder;
class PIDController;

class MecanumDrive : public RobotDrive {
public:
    MecanumDrive(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
                SpeedController *frontRightMotor, SpeedController *rearRightMotor);
    MecanumDrive(SpeedController &frontLeftMotor, SpeedController &rearLeftMotor,
                SpeedController &frontRightMotor, SpeedController &rearRightMotor);

    virtual ~MecanumDrive();

    typedef enum {
        Omni = 0,
        Strafe,
        Arcade,
        FLpivot,
        FRpivot,
        RLpivot,
        RRpivot
    } DriveMode;

    /* This function assumes field-oriented driving
     * x is the magnitude of x translation of the robot [-1..1]
     * y is the magnitude of y translation of the robot [-1..1]
     * rotation is the magnitude of rotation [-1..1]
     */
    void Drive( float x , float y , float rotation , float gyroAngle = 0.0 );

    /* If enabled, squares joystick inputs for fine-tuned driving at low speeds
     * while maintaining max speed
     */
    void SquareInputs( bool squared );

    void SetDriveMode( DriveMode mode );
    DriveMode GetDriveMode();

    void SetEncoderPorts( UINT32 flA , UINT32 flB , UINT32 rlA , UINT32 rlB ,
            UINT32 frA , UINT32 frB , UINT32 rrA , UINT32 rrB );

    // Makes class use encoders to keep robot driving properly
    void EnableEncoders( bool pidEnabled );

    // Returns true if encoders are enabled
    bool AreEncodersEnabled();

    // Returns encoder rates if the encoders are enabled
    double GetFLrate();
    double GetRLrate();
    double GetFRrate();
    double GetRRrate();

    // Returns encoder distances if the encoders are enabled
    double GetFLdist();
    double GetRLdist();
    double GetFRdist();
    double GetRRdist();

private:
    DISALLOW_COPY_AND_ASSIGN(MecanumDrive);

    static float maxWheelSpeed;

    bool m_squaredInputs;
    DriveMode m_driveMode;

    Encoder* m_flEncoder;
    UINT32 m_flA; // front left encoder, DIO channel A
    UINT32 m_flB; // front left encoder, DIO channel B
    PIDController* m_flPID;

    Encoder* m_rlEncoder;
    UINT32 m_rlA;
    UINT32 m_rlB;
    PIDController* m_rlPID;

    Encoder* m_frEncoder;
    UINT32 m_frA;
    UINT32 m_frB;
    PIDController* m_frPID;

    Encoder* m_rrEncoder;
    UINT32 m_rrA;
    UINT32 m_rrB;
    PIDController* m_rrPID;

    bool m_pidEnabled;
};

#endif // MECANUM_DRIVE_HPP
