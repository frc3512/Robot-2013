//=============================================================================
//File Name: LiveGrapherClient.cpp
//Description: Sends data to a remote location to be graphed
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "LiveGrapherClient.hpp"

LiveGrapherClient* LiveGrapherClient::m_instance = NULL;

LiveGrapherClient* LiveGrapherClient::getInstance() {
    if ( m_instance == NULL ) {
        m_instance = new LiveGrapherClient( 1150 );
    }

    return m_instance;
}

LiveGrapherClient::LiveGrapherClient( unsigned short portNumber ) {

}
