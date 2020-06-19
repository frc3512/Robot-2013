// Copyright (c) 2013-2020 FRC Team 3512. All Rights Reserved.

#include "GeartoothEncoder.hpp"

GeartoothEncoder::GeartoothEncoder(int channel, int teethPerRevolution,
                                   double gearRatio)
    : m_counter{channel},
      m_teethPerRevolution{teethPerRevolution},
      m_gearRatio{gearRatio} {
    m_counter.SetSamplesToAverage(5);
}

units::revolutions_per_minute_t GeartoothEncoder::GetRate() const {
    // Derivation of RPM:
    // gear seconds per tick = period
    // gear ticks per second = 1 / period
    // gear ticks per minute = 60 / period
    // gear RPM = 60 / (kTicksPerRev * period)
    // shooter wheel RPM = kGearRatio * 60 / (kTicksPerRev * period)
    return units::revolutions_per_minute_t{
        m_gearRatio * 60.0 / (m_teethPerRevolution * m_counter.GetPeriod())};
}
