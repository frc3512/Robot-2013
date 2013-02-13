//=============================================================================
//File Name: LiveGrapherHost.cpp
//Description: Sends data to a remote location to be graphed
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "LiveGrapherHost.hpp"

LiveGrapherHost* LiveGrapherHost::m_instance = NULL;

LiveGrapherHost* LiveGrapherHost::getInstance() {
    if ( m_instance == NULL ) {
        m_instance = new LiveGrapherHost( 1150 );
    }

    return m_instance;
}

LiveGrapherHost::LiveGrapherHost( unsigned short portNumber ) {

}
