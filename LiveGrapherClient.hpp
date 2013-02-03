//=============================================================================
//File Name: LiveGrapherClient.hpp
//Description: Sends data to a remote location to be graphed
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef LIVE_GRAPHER_CLIENT_HPP
#define LIVE_GRAPHER_CLIENT_HPP

#include "SFML/Network/UdpSocket.hpp"
#include "SFML/System/NonCopyable.hpp"

class LiveGrapherClient : public sf::NonCopyable {
public:
    LiveGrapherClient* getInstance();

private:
    // Disallow copy and assign
    LiveGrapherClient( const LiveGrapherClient& );
    void operator=( const LiveGrapherClient& );

    explicit LiveGrapherClient( unsigned short portNumber );

    static LiveGrapherClient* m_instance;
};

#endif // LIVE_GRAPHER_CLIENT_HPP
