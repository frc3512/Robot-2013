//=============================================================================
//File Name: MecanumDrive.hpp
//Description: Drives the robot with Mecanum wheels in Tank drive configuration
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef MECANUM_DRIVE_HPP
#define MECANUM_DRIVE_HPP

#include <RobotDrive.h>
#include "../Settings.hpp"

class Encoder;
class PIDController;

class MecanumDrive : public RobotDrive {
public:
    MecanumDrive(SpeedController &frontLeftMotor, SpeedController &rearLeftMotor,
                SpeedController &frontRightMotor, SpeedController &rearRightMotor,
                UINT32 flA, UINT32 flB, UINT32 rlA, UINT32 rlB,
                UINT32 frA, UINT32 frB, UINT32 rrA, UINT32 rrB);

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

    // Sets joystick deadband
    void SetDeadband( float band );

    void SetDriveMode( DriveMode mode );
    DriveMode GetDriveMode();

    // Makes class use encoders to keep robot driving properly
    void EnableEncoders( bool pidEnabled );

    // Returns true if encoders are enabled
    bool AreEncodersEnabled();

    // Set encoder distances to 0
    void ResetEncoders();

    // Reload PID constants
    void ReloadPID();

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

    // Returns encoder PID loop setpoints
    double GetFLsetpoint();
    double GetRLsetpoint();
    double GetFRsetpoint();
    double GetRRsetpoint();

    const static float maxWheelSpeed;

private:
    DISALLOW_COPY_AND_ASSIGN(MecanumDrive);

    Settings m_settings;

    bool m_squaredInputs;
    float m_deadband;
    DriveMode m_driveMode;

    Encoder* m_flEncoder;
    PIDController* m_flPID;

    Encoder* m_rlEncoder;
    PIDController* m_rlPID;

    Encoder* m_frEncoder;
    PIDController* m_frPID;

    Encoder* m_rrEncoder;
    PIDController* m_rrPID;

    bool m_pidEnabled;
};

#endif // MECANUM_DRIVE_HPP
