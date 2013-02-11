//=============================================================================
//File Name: Autonomous.cpp
//Description: Initializes timer, then calls the appropriate autonomous routine
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "OurRobot.hpp"

void OurRobot::Autonomous() {
    mainCompressor.Start();
    shooterEncoder.Start();

    autoTime.Reset();
    autoTime.Start();

    autonModes.execAutonomous( autonMode );

    autoTime.Stop();
}
