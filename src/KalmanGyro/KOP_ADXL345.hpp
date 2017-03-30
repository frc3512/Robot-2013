// Copyright (c) FRC Team 3512, Spartatroniks 2013-2017. All Rights Reserved.

#pragma once

#include "GyroBase.hpp"

class AnalogModule;
class I2C;

class KOP_ADXL345 : public GyroBase {
public:
    KOP_ADXL345(UINT32 gyroSlot, UINT32 gyroChannel, UINT32 i2cSlot,
                UINT32 accelAddress, UINT32 magnetAddress);
    virtual ~KOP_ADXL345();

private:
    // KOP 1-axis gyro
    AnalogModule* m_gyro;
    UINT32 m_gyroChannel;

    // ADXL345 3-axis accelerometer
    I2C* m_accel;

    // MAG3110 3-axis magnetometer
    I2C* m_magnet;

    /* Read raw y-axis from gyro
     * Actually measures y-axis of gyro for consistency with accelerometer
     */
    int readGyroX();

    // Read raw data from accelerometer
    int readAccelX();
    int readAccelY();
    int readAccelZ();

    // Read raw data from magnetometer
    int readMagnetX();
    int readMagnetY();

    // Return zero points for gyro
    double getGyroXzero();

    // Return zero points for accelerometer
    double getAccelXzero();
    double getAccelYzero();
    double getAccelZzero();

    // Return zero points for magnetometer
    double getMagnetXzero();
    double getMagnetYzero();

    // Return sensitivity of gyro in volts per degree/second
    double getGyroLSBsPerUnit();
};
