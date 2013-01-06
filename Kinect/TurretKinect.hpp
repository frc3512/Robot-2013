//=============================================================================
//File Name: TurretKinect.hpp
//Description: Declares Kinect for aiming turret
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef TURRET_KINECT_HPP
#define TURRET_KINECT_HPP

#include "KinectBase.hpp"

class TurretKinect : public KinectBase {
public:
    static const unsigned int pxlDeadband = 5;

    TurretKinect( sf::IpAddress IP , unsigned short portNumber );

    signed short getPixelOffset();
    unsigned int getDistance();
    signed char getTargetSelect();

    void setPixelOffset( signed short var );
    void setDistance( unsigned int var );
    void setTargetSelect( signed char var );

protected:
    // derived definitions of packet manipulation functions
    void insertPacketMutexless( PacketStruct& insertHere );
    void extractPacketMutexless( PacketStruct& extractHere );
    void clearValuesMutexless();

private:
    // Packet data
    signed short pixelOffset; // amount of pixels turret is off of center (is from -320 to 320)
    unsigned int distance;
    signed char targetSelect;
};

#endif // TURRET_KINECT_HPP
