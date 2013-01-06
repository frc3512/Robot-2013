//=============================================================================
//File Name: TurretKinect.cpp
//Description: Declares Kinect for aiming turret
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "TurretKinect.hpp"

const unsigned int TurretKinect::pxlDeadband;

TurretKinect::TurretKinect( sf::IpAddress IP , unsigned short portNumber ) : KinectBase( IP , portNumber ) {
    clearValues();
}

signed short TurretKinect::getPixelOffset() {
    signed short temp;

    valueMutex.lock();
    temp = pixelOffset;
    valueMutex.unlock();

    return temp;
}

unsigned int TurretKinect::getDistance() {
    signed short temp;

    valueMutex.lock();
    temp = distance;
    valueMutex.unlock();

    return temp;
}

signed char TurretKinect::getTargetSelect() {
    signed short temp;

    valueMutex.lock();
    temp = targetSelect;
    valueMutex.unlock();

    return temp;
}

void TurretKinect::setPixelOffset( signed short var ) {
    valueMutex.lock();
    pixelOffset = var;
    valueMutex.unlock();
}

void TurretKinect::setDistance( unsigned int var ) {
    valueMutex.lock();
    distance = var;
    valueMutex.unlock();
}

void TurretKinect::setTargetSelect( signed char var ) {
    valueMutex.lock();
    targetSelect = var;
    valueMutex.unlock();
}

void TurretKinect::insertPacketMutexless( PacketStruct& insertHere ) {
    insertHere.packet << targetSelect;
}

void TurretKinect::extractPacketMutexless( PacketStruct& extractHere ) {
    extractHere.packet >> pixelOffset >> distance >> targetSelect;
}

void TurretKinect::clearValuesMutexless() {
    pixelOffset = 0;
    distance = 0;
    targetSelect = 0;
}
