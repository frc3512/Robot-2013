// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Counter.h>
#include <units/units.h>

/**
 * This class counts the number of gear teeth which have passed using a Counter
 * and Hall's Effect sensor plugged into a DIO channel. It returns the RPM of
 * the shooter wheel given the gear ratio and number of teeth on the gear.
 */
class GeartoothEncoder {
public:
    GeartoothEncoder(int channel, int teethPerRevolution, double gearRatio);

    /**
     * Returns angular velocity of shooter sheel.
     */
    units::revolutions_per_minute_t GetRate() const;

private:
    // Counts number of pulses from Hall effect sensor
    frc::Counter m_counter;

    // Number of teeth per revolution of gear
    int m_teethPerRevolution;

    // Conversion factor between RPM of gear and RPM of shooter wheel
    double m_gearRatio;
};
