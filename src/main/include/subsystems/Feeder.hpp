// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Solenoid.h>
#include <frc2/Timer.h>
#include <units/units.h>

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
    /**
     * Starts process of pushing frisbee into shooter.
     */
    void Activate();

    /**
     * Returns true if this instance is in the middle of feeding frisbees.
     */
    bool IsFeeding();

    /**
     * Continues transition of feeder state.
     */
    void Update();

private:
    frc::Solenoid m_frisbeeFeed{1};
    frc2::Timer m_feedTimer;

    // Time it takes for the feed actuator to completely switch states
    units::second_t m_feedDelay = 0.3_s;

    frc::Solenoid m_frisbeeGuard{3};
    frc2::Timer m_guardTimer;

    // Time it takes for the frisbee to pass into the shooter after the feed
    // actuator fully contracts
    units::second_t m_guardDelay = 0.3_s;

    bool m_isActivated = false;

    // Number of frisbees shot since feeder was last activated
    unsigned int m_numShot = 0;

    // Number of frisbees to shoot since feeder was last activated
    unsigned int m_totalToShoot = 0;
};
