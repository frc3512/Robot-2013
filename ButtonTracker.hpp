//=============================================================================
//File Name: ButtonTracker.hpp
//Description: Helps user determine if joystick button was just pressed or just
//             released
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef BUTTON_TRACKER_HPP
#define BUTTON_TRACKER_HPP

/* This class allows you to check if a button was pressed or released without
 * having to wait in one spot of code until that happens.
 *
 * It is useful for situations in which you need to toggle a variable and just
 * checking for it with Joystick::GetRawButton(UINT32) would cause it to toggle
 * in every iteration of a loop.
 *
 * USAGE
 * 1) Call updateButtons() at beginning of loop to get new button statuses from
 *    the Driver Station
 * 2) Call pressedButton(UINT32) or releasedButton(UINT32) to poll for whether
 *    the button was pressed or released since last loop iteration
 *
 * None of these functions block.
 */

class DriverStation;

class ButtonTracker {
public:
    ButtonTracker( UINT32 port );

    void updateButtons(); // gets new button statuses for joystick from Driver Station

    bool pressedButton( UINT32 button ); // returns true if button wasn't pressed but is now
    bool releasedButton( UINT32 button ); // returns true if button was pressed but isn't now

protected:
    UINT32 m_port;

private:
    static bool getButtonState( short& buttonStates , UINT32& button );

    static bool m_driverStationInit;
    static DriverStation* m_driverStation;

    short m_oldStates;
    short m_newStates;
};

#endif // BUTTON_TRACKER_H
