// Copyright (c) FRC Team 3512, Spartatroniks 2013-2017. All Rights Reserved.

#pragma once

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
    Feeder(uint32_t feedSolenoid, uint32_t guardSolenoid, double feedDelay,
           double guardDelay);
    ~Feeder();

    // Starts process of pushing frisbee into shooter
    void activate();

    // Continues transition of feeder state
    void update();

    // Returns true if this instance is in the middle of feeding frisbees
    bool isFeeding();

    /* Sets delay for which the plunging actuator stays retracted (to give the
     * actuator time to complete its transit and to wait for frisbees to fall
     * into position)
     */
    void setFeedDelay(double time);

    /* Sets delay for which the guard actuator stays down (to allow frisbees to
     * pass into the shooter)
     */
    void setGuardDelay(double time);

private:
    Solenoid m_frisbeeFeed;
    Timer m_feedTimer;

    // Time it takes for the feed actuator to completely switch states
    double m_feedDelay;

    Solenoid m_frisbeeGuard;
    Timer m_guardTimer;

    /* Time it takes for the frisbee to pass into the shooter after the feed
     * actuator fully contracts
     */
    double m_guardDelay;

    bool m_isActivated;

    // Number of frisbees shot since feeder was last activated
    unsigned int m_numShot;

    // Number of frisbees to shoot since feeder was last activated
    unsigned int m_totalToShoot;
};
