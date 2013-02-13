//=============================================================================
//File Name: LiveGrapherHost.hpp
//Description: Sends data to a remote location to be graphed
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef LIVE_GRAPHER_HOST_HPP
#define LIVE_GRAPHER_HOST_HPP

#include "SFML/Network/UdpSocket.hpp"
#include "SFML/System/NonCopyable.hpp"

class LiveGrapherHost : public sf::NonCopyable {
public:
    LiveGrapherHost* getInstance();

private:
    // Disallow copy and assign
    LiveGrapherHost( const LiveGrapherHost& );
    void operator=( const LiveGrapherHost& );

    explicit LiveGrapherHost( unsigned short portNumber );

    static LiveGrapherHost* m_instance;
};

#endif // LIVE_GRAPHER_HOST_HPP
