//=============================================================================
//File Name: DriveKinect.hpp
//Description: Declares Kinect for driving robot
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef DRIVE_KINECT_HPP
#define DRIVE_KINECT_HPP

#include "KinectBase.hpp"

class DriveKinect : public KinectBase {
public:
    DriveKinect( sf::IpAddress IP , unsigned short portNumber );

    float getRight();
    float getLeft();

    void setRight( float var );
    void setLeft( float var );

protected:
    // derived definitions of packet manipulation functions
    void insertPacketMutexless( PacketStruct& insertHere );
    void extractPacketMutexless( PacketStruct& extractHere );
    void clearValuesMutexless();

private:
    // Packet data
    float right;
    float left;
};

#endif // DRIVE_KINECT_HPP
