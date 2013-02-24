//=============================================================================
//File Name: Feeder.hpp
//Description: Lowers guard and pushes frisbees into the shooter
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef FEEDER_HPP
#define FEEDER_HPP

#include <Solenoid.h>
#include <Timer.h>

/* Notes:
 *
 * For each solenoid, 'false' represents the default position of the actuator.
 * The feed actuator is in by default; the guard actuator is up by default.
 *
 * Delays are used here so the actuators have time to fully activate before
 * retracting them again.
 */

class Feeder {
public:
    /* 'feedSolenoid' is for the actuator feeding frisbees into the shooter
     * 'guardSolenoid' is for the acuator preventing frisbees from entering the
     *     shooter prematurely
     */
    Feeder( UINT32 feedSolenoid , UINT32 guardSolenoid ,
            double feedDelay , double guardDelay );
    ~Feeder();

    // Starts process of pushing frisbee into shooter
    void activate();

    // Continues transition of feeder state
    void update();

    // Returns true if this instance is in the middle of feeding a frisbee
    bool isFeeding();

    /* Sets delay for which the plunging actuator stays retracted (to give the
     * actuator time to complete its transit and to wait for frisbees to fall
     * into position)
     */
    void setFeedDelay( double time );

    /* Sets delay for which the guard actuator stays down (to allow frisbees to
     * pass into the shooter)
     */
    void setGuardDelay( double time );

private:
    Solenoid m_frisbeeFeed;
    Timer m_feedTimer;
    double m_feedDelay;

    Solenoid m_frisbeeGuard;
    Timer m_guardTimer;
    double m_guardDelay;

    bool m_isActivated;
};

#endif // FEEDER_HPP
