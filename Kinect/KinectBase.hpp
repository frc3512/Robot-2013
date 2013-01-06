//=============================================================================
//File Name: KinectBase.hpp
//Description: Defines interface for all types of Kinect usage
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef KINECT_BASE_HPP
#define KINECT_BASE_HPP

#include "../SFML/Network/IpAddress.hpp"
#include "../SFML/Network/Packet.hpp"
#include "../SFML/Network/UdpSocket.hpp"
#include "../SFML/System/Clock.hpp"
#include "../SFML/System/Mutex.hpp"
#include "../SFML/System/Thread.hpp"

typedef struct PacketStruct {
    sf::Packet packet;
    sf::Mutex mutex;
} PacketStruct;

class KinectBase {
public:
    KinectBase( sf::IpAddress IP , unsigned short portNumber ); // takes IP and portNumber to which to bind
    virtual ~KinectBase();

    sf::Socket::Status getOnlineStatus();
    void setOnlineStatus( sf::Socket::Status var );

    void send(); // sends data to single board computer

    /* Zeroes all data in the packet (has mutex wrapping)
     * Used if socket connection fails or data is old
     */
    void clearValues();

protected:
    /* The next two methods are used for packing a derived Kinect class's packet.
     * Each Kinect's packet is unique, so the method must be defined by the derived class.
     */

    // Packs data into user-provided packet for sending to single board computer (NO MUTEXES)
    virtual void insertPacketMutexless( PacketStruct& insertHere ) = 0;

    // Unpacks data received from single board computer into user-provided packet (NO MUTEXES)
    virtual void extractPacketMutexless( PacketStruct& extractHere ) = 0;

    // Zeroes all data in the packet
    virtual void clearValuesMutexless() = 0;

    // Used for locking a packet during send, receive, and manipulation functions
    PacketStruct receiver;
    PacketStruct sender;

    // locks data received from packet
    sf::Mutex valueMutex;

private:
    // tells the receive thread to exit when the object instance is destructed
    volatile bool closeThread;

    sf::Thread socketThread;

    sf::UdpSocket kinectSocket;
    sf::UdpSocket sendSocket;

    sf::IpAddress sendIP; // IP address of data receiver when sending
    unsigned short sendPort; // stores port of remote sender when data is received

    sf::IpAddress receiveIP;
    unsigned short receivePort;

    sf::Clock valueAge; // used to throw away old values

    sf::Socket::Status onlineStatus;

    void receive(); // receives data from on-board computer

    // Packs data into sendPacket for sending to single board computer (has mutex wrapping)
    void insertPacket( PacketStruct& insertHere );

    // Unpacks data from packet received from single board computer (has mutex wrapping)
    void extractPacket( PacketStruct& extractHere );
};

#endif // KINECT_BASE_HPP
