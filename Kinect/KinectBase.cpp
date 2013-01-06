//=============================================================================
//File Name: KinectBase.cpp
//Description: Defines interface for all types of Kinect usage
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "KinectBase.hpp"
#include <Timer.h>

KinectBase::KinectBase( sf::IpAddress IP , unsigned short portNumber ) :
        socketThread( &KinectBase::receive , this ) , sendIP( IP ) , sendPort( portNumber ) {
	closeThread = false;
    kinectSocket.bind( portNumber );
    kinectSocket.setBlocking( false );
	socketThread.launch();
}

KinectBase::~KinectBase() {
	closeThread = true;

	socketThread.wait(); // waits for receive thread to end
	kinectSocket.unbind();
}

sf::Socket::Status KinectBase::getOnlineStatus() {
    sf::Socket::Status temp;

    valueMutex.lock();
    temp = onlineStatus;
    valueMutex.unlock();

    return temp;
}

void KinectBase::setOnlineStatus( sf::Socket::Status var ) {
    valueMutex.lock();
    onlineStatus = var;
    valueMutex.unlock();
}

void KinectBase::send() {
    insertPacket( sender ); // inserts data into the sender packet

    sender.mutex.lock();
    sendSocket.send( sender.packet , sendIP , sendPort );
	sender.mutex.unlock();
}

void KinectBase::receive() {
	sf::Socket::Status sOnlineStatus; // used to store safe value for Kinect's packet data status

	while ( !closeThread ) {
	    receiver.mutex.lock();
		onlineStatus = kinectSocket.receive( receiver.packet , receiveIP , receivePort );
		receiver.mutex.unlock();

		// get a safe version of onlineStatus
		valueMutex.lock();
		sOnlineStatus = onlineStatus;
		valueMutex.unlock();

		// if data received
		if ( sOnlineStatus == sf::Socket::Done ) {
			extractPacket( receiver ); // unpack new data from receiver packet
			valueAge.restart();
		}
		// if socket failed
		else if ( sOnlineStatus == sf::Socket::Disconnected || sOnlineStatus == sf::Socket::Error ) {
			clearValues();
		}
		// if values received are older than 600ms, clear them
		else if ( valueAge.getElapsedTime().asMilliseconds() > 600 ) {
			clearValues();
		}

		// else do nothing because the socket is fine; it just doesn't have any new values

		// Keeps non-blocking socket thread from consuming all system resources
		Wait( 0.1 );
	}
}

void KinectBase::insertPacket( PacketStruct& insertHere ) {
    insertHere.mutex.lock();

    insertHere.packet.clear();

    valueMutex.lock();

    insertPacketMutexless( sender ); // inserts data into the sender packet

    valueMutex.unlock();
    insertHere.mutex.unlock();
}

void KinectBase::extractPacket( PacketStruct& extractHere ) {
    extractHere.mutex.lock();
    valueMutex.lock();

    extractPacketMutexless( receiver ); // extracts data from the receiver packet

    valueMutex.unlock();
    extractHere.mutex.unlock();
}

void KinectBase::clearValues() {
	valueMutex.lock();

	clearValuesMutexless();
	valueAge.restart(); // there are new values, so reset the timer

	valueMutex.unlock();
}
