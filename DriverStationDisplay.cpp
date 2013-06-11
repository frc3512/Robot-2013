//=============================================================================
//File Name: DriverStationDisplay.cpp
//Description: Receives IP address from remote host then sends HUD data there
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "DriverStationDisplay.hpp"
#include <cstdlib>
#include <cstring>

DriverStationDisplay* DriverStationDisplay::m_dsDisplay = NULL;

class DriverStationDisplayInit {
public:
    DriverStationDisplayInit();
    ~DriverStationDisplayInit();
};

DriverStationDisplayInit driverStationDisplayInit;

DriverStationDisplayInit::DriverStationDisplayInit() {
}

DriverStationDisplayInit::~DriverStationDisplayInit() {
    delete DriverStationDisplay::m_dsDisplay;
}

DriverStationDisplay::~DriverStationDisplay() {
    m_socket.unbind();
    std::free( m_recvBuffer );
}

DriverStationDisplay* DriverStationDisplay::getInstance( unsigned short dsPort ) {
    if ( m_dsDisplay == NULL ) {
        m_dsDisplay = new DriverStationDisplay( dsPort );
    }

    return m_dsDisplay;
}

void DriverStationDisplay::sendToDS( sf::Packet* userData ) {
    if ( m_dsIP != sf::IpAddress::None ) {
        if ( userData == NULL ) {
            m_socket.send( *static_cast<sf::Packet*>(this) , m_dsIP , m_dsPort );
        }
        else {
            m_socket.send( *userData , m_dsIP , m_dsPort );
        }
    }

    // Used for testing purposes
    sf::IpAddress testIP( 10 , 35 , 12 , 42 );
    if ( userData == NULL ) {
        m_socket.send( *static_cast<sf::Packet*>(this) , testIP , m_dsPort );
    }
    else {
        m_socket.send( *userData , testIP , m_dsPort );
    }
}

const std::string DriverStationDisplay::receiveFromDS( void* userData ) {
    if ( m_socket.receive( m_recvBuffer , 256 , m_recvAmount , m_recvIP , m_recvPort ) == sf::Socket::Done ) {
        if ( std::strncmp( m_recvBuffer , "connect\r\n" , 9 ) == 0 ) {
            m_dsIP = m_recvIP;
            m_dsPort = m_recvPort;

            return "connect\r\n";
        }
        else if ( std::strncmp( m_recvBuffer , "autonSelect\r\n" , 13 ) == 0 ) {
            // Next byte after command is selection choice
            *static_cast<char*>(userData) = m_recvBuffer[13];

            return "autonSelect\r\n";
        }
    }

    return "NONE";
}

DriverStationDisplay::DriverStationDisplay( unsigned short portNumber ) : m_dsIP( sf::IpAddress::None ) , m_dsPort( portNumber ) {
    m_socket.bind( portNumber );
    m_socket.setBlocking( false );
    m_recvIP = sf::IpAddress( 0 , 0 , 0 , 0 );
    m_recvPort = 0;
    m_recvBuffer = static_cast<char*>( std::malloc( 256 ) );
}
