// Copyright (c) 2013-2017 FRC Team 3512. All Rights Reserved.

#include "ButtonTracker.hpp"

#include <DriverStation.h>

bool ButtonTracker::m_driverStationInit = false;
DriverStation* ButtonTracker::m_driverStation = NULL;

ButtonTracker::ButtonTracker(uint32_t port) {
    if (!m_driverStationInit) {
        m_driverStation = DriverStation::GetInstance();
        m_driverStationInit = true;
    }

    m_port = port;
    m_oldStates = 0;
    m_newStates = 0;
}

void ButtonTracker::updateButtons() {
    m_oldStates = m_newStates;  // "new" values are now "old"
    m_newStates =
        m_driverStation->GetStickButtons(m_port);  // get even newer values
}

bool ButtonTracker::pressedButton(uint32_t button) {
    return getButtonState(m_oldStates, button) ==
               false &&  // if button wasn't pressed
           getButtonState(m_newStates, button) == true;  // and it is now
}

bool ButtonTracker::releasedButton(uint32_t button) {
    return getButtonState(m_oldStates, button) ==
               true &&  // if button was pressed
           getButtonState(m_newStates, button) == false;  // and it isn't now
}

bool ButtonTracker::getButtonState(int16_t& buttonStates, uint32_t& button) {
    return ((0x1 << (button - 1)) & buttonStates) != 0;
}
