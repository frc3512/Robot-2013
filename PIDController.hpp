//=============================================================================
//File Name: PIDController.hpp
//Description: Implements a PID controller for use with position and velocity
//             control
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

/* We modified WPILib's PIDController class because it didn't multiply the PID
 * constants by the loop's dt; a change in sample rate produces different
 * behavior with the same constants.
 */

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <atomic>
#include <Base.h>
#include <semLib.h>
#include <Controller.h>
#include <Timer.h>

class PIDOutput;
class PIDSource;
class Notifier;

/**
 * Class implements a PID Control Loop.
 *
 * Creates a separate thread which reads the given PIDSource and takes
 * care of the integral calculations, as well as writing the given
 * PIDOutput
 */
class PIDController : public Controller
{
public:
    PIDController(float p, float i, float d,
                    PIDSource *source, PIDOutput *output,
                    float period = 0.05);
    PIDController(float p, float i, float d, float f,
                    PIDSource *source, PIDOutput *output,
                    float period = 0.05);
    virtual ~PIDController();
    virtual float Get();
    virtual void SetContinuous(bool continuous = true);
    virtual void SetInputRange(float minimumInput, float maximumInput);
    virtual void SetOutputRange(float mimimumOutput, float maximumOutput);
    virtual void SetPID(float p, float i, float d);
    virtual void SetPID(float p, float i, float d, float f);
    virtual float GetP();
    virtual float GetI();
    virtual float GetD();
    virtual float GetF();

    virtual void SetSetpoint(float setpoint);
    virtual float GetSetpoint();

    virtual float GetError();

    virtual void SetTolerance(float percent);
    virtual void SetAbsoluteTolerance(float absValue);
    virtual void SetPercentTolerance(float percentValue);
    virtual bool OnTarget();

    virtual void Enable();
    virtual void Disable();
    virtual bool IsEnabled();

    virtual void Reset();

private:
    float m_P;            // factor for "proportional" control
    float m_I;            // factor for "integral" control
    float m_D;            // factor for "derivative" control
    float m_F;            // factor for "feed forward" control
    float m_maximumOutput;    // |maximum output|
    float m_minimumOutput;    // |minimum output|
    float m_maximumInput;        // maximum input - limit setpoint to this
    float m_minimumInput;        // minimum input - limit setpoint to this
    bool m_continuous;    // do the endpoints wrap around? eg. Absolute encoder
    bool m_enabled;             //is the pid controller enabled
    float m_prevError;    // the prior sensor input (used to compute velocity)
    double m_totalError; //the sum of the errors for use in the integral calc
    enum {kAbsoluteTolerance, kPercentTolerance, kNoTolerance} m_toleranceType;
    float m_tolerance;    //the percetage or absolute error that is considered on target
    float m_setpoint;
    float m_error;
    float m_result;
    float m_period;

    std::atomic<float> m_deltaError;

    SEM_ID m_semaphore;

    PIDSource *m_pidInput;
    PIDOutput *m_pidOutput;
    Notifier *m_controlLoop;
    Timer m_loopTimer;

    void Initialize(float p, float i, float d, float f,
                    PIDSource *source, PIDOutput *output,
                    float period = 0.05);
    static void CallCalculate(void *controller);
    void Calculate();

protected:
    DISALLOW_COPY_AND_ASSIGN(PIDController);
};

#endif // PID_CONTROLLER_HPP
