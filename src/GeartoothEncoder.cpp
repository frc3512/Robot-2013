// Copyright (c) 2013-2017 FRC Team 3512. All Rights Reserved.

#include "GeartoothEncoder.hpp"

GeartoothEncoder::GeartoothEncoder(UINT32 channel, UINT32 teeth,
                                   float gearRatio)
    : m_counter(channel),
      m_rpmFilter(0, 0),
      m_latestRPM(0),
      m_gearRatio(gearRatio),
      m_teeth(teeth) {
    m_sampleThread = new Notifier(&GeartoothEncoder::threadFunc, this);
    m_sampleThread->StartPeriodic(0.1f);
}

GeartoothEncoder::~GeartoothEncoder() { delete m_sampleThread; }

void GeartoothEncoder::start() { m_counter.Start(); }

void GeartoothEncoder::stop() { m_counter.Stop(); }

void GeartoothEncoder::setTeethPerRevolution(UINT32 teeth) {
    m_dataMutex.take();

    m_teeth = teeth;

    m_dataMutex.give();
}

void GeartoothEncoder::setGearRatio(float ratio) {
    m_dataMutex.take();

    m_gearRatio = ratio;

    m_dataMutex.give();
}

float GeartoothEncoder::getFilterRPM() { return m_rpmFilter.getEstimate(); }

float GeartoothEncoder::getCurrentRPM() {
    m_dataMutex.take();

    float latestRPM = m_latestRPM;

    m_dataMutex.give();

    return latestRPM;
}

void GeartoothEncoder::setFilterQ(double Q) { m_rpmFilter.setQ(Q); }

void GeartoothEncoder::setFilterR(double R) { m_rpmFilter.setR(R); }

void GeartoothEncoder::resetFilter() { m_rpmFilter.reset(); }

void GeartoothEncoder::setSampleRate(UINT32 sampleRate) {
    m_sampleThread->StartPeriodic(1.f / sampleRate);
}

void GeartoothEncoder::threadFunc(void* object) {
    GeartoothEncoder* encoderObj = static_cast<GeartoothEncoder*>(object);

    /* Derivation of RPM:
     * period = seconds per tick of gear
     * 1 / period = ticks per second of gear
     * 60 / period = ticks per minute of gear
     * 56 ticks per revolution therefore: 60 / ( 56 * period ) = revolutions per
     * minute of gear
     * 4 * 60 / ( 56 * period ) = revolutions per minute of shooter wheel
     *
     * shooterRPM = 4.f * 60.f / ( 56.f * shooterEncoder.GetPeriod() );
     */

    encoderObj->m_dataMutex.take();

    // Store latest RPM
    encoderObj->m_latestRPM =
        encoderObj->m_gearRatio * 60.f /
        (encoderObj->m_teeth * encoderObj->m_counter.GetPeriod());

    // Add RPM value to Kalman filter
    encoderObj->m_rpmFilter.update(encoderObj->m_latestRPM);

    encoderObj->m_dataMutex.give();
}
