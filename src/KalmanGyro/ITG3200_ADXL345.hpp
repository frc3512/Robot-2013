// Copyright (c) FRC Team 3512, Spartatroniks 2013-2017. All Rights Reserved.

#pragma once

#include <DigitalModule.h>

#include "GyroBase.hpp"

class I2C;

class ITG3200_ADXL345 : public GyroBase {
public:
    ITG3200_ADXL345(UINT32 slot, UINT32 gyroAddress, UINT32 accelAddress,
                    UINT32 magnetAddress);
    virtual ~ITG3200_ADXL345();

private:
    // ITG3200 3-axis gyro
    I2C* m_gyro;

    // ADXL345 3-axis accelerometer
    I2C* m_accel;

    // MAG3110 3-axis magnetometer
    I2C* m_magnet;

    /* Read raw y-axis from gyro
     * Actually measures y-axis of gyro for consistency with accelerometer
     */
    int readGyroX();

    /* Read raw x-axis from gyro
     * Actually measures x-axis of gyro for consistency with accelerometer
     */
    int readGyroY();

    // Read raw z-axis from gyro
    int readGyroZ();

    // Read raw data from accelerometer
    int readAccelX();
    int readAccelY();
    int readAccelZ();

    // Read raw data from magnetometer
    int readMagnetX();
    int readMagnetY();

    // Return zero points for gyro
    double getGyroXzero();
    double getGyroYzero();
    double getGyroZzero();

    // Return zero points for accelerometer
    double getAccelXzero();
    double getAccelYzero();
    double getAccelZzero();

    // Return zero points for magnetometer
    double getMagnetXzero();
    double getMagnetYzero();

    // Return sensitivity of gyro in least significant bits per degree/second
    double getGyroLSBsPerUnit();
};
