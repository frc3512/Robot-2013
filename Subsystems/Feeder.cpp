//=============================================================================
//File Name: Feeder.cpp
//Description: Lowers guard and pushes frisbees into the shooter
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "Feeder.hpp"

Feeder::Feeder( UINT32 feedSolenoid , UINT32 guardSolenoid ,
        double feedDelay , double guardDelay ) :
        m_frisbeeFeed( feedSolenoid ) ,
        m_feedDelay( feedDelay ) ,
        m_frisbeeGuard( guardSolenoid ) ,
        m_guardDelay( guardDelay ) {
}

Feeder::~Feeder() {

}

void Feeder::activate() {
    if ( !m_isActivated ) {
        // Pull feeder in for pushing frisbee
        m_frisbeeFeed.Set( true );

        // Lower shooter guard so frisbees can leave
        m_frisbeeGuard.Set( true );

        // Start the delay timers
        m_feedTimer.Start();
        m_guardTimer.Start();
    }
}

void Feeder::setFeedDelay( double time ) {
    m_feedDelay = time;
}

void Feeder::setGuardDelay( double time ) {
    m_guardDelay = time;
}

void Feeder::update() {
    /* If frisbee is going to be fed into the shooter and the actuator had
     * enough time to move completely out from underneath the frisbees
     */
    if ( m_isActivated ) {
        if ( m_frisbeeFeed.Get() && m_feedTimer.Get() > m_feedDelay ) {
            // Reset feed actuator and push frisbee into shooter
            m_frisbeeFeed.Set( false );

            // Reset timer
            m_feedTimer.Stop();
            m_feedTimer.Reset();
        }

        // If frisbee guard is down and frisbee has moved past guard
        if ( m_frisbeeGuard.Get() == true && m_guardTimer.Get() > m_guardDelay ) {
            m_frisbeeGuard.Set( false );

            // Reset timer
            m_guardTimer.Stop();
            m_guardTimer.Reset();

            // Process is done, allow it to repeat
            m_isActivated = false;
        }
    }
}
