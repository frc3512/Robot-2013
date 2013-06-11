//=============================================================================
//File Name: KOP_ADXL345.hpp
//Description: Implements a Kalman filter with the KOP 1-axis gyro and the
//             ADXL345 3-axis accelerometer
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef KOP_ADXL345_HPP
#define KOP_ADXL345_HPP

#include "GyroBase.hpp"
#include "KopGyro.hpp"

#include <SensorBase.h>
#include <DigitalModule.h>

class I2C;

class KOP_ADXL345 : public GyroBase , public SensorBase {
public:
    KOP_ADXL345( UINT32 gyroSlot , UINT32 gyroChannel , UINT32 accelSlot , UINT32 accelAddress );
    virtual ~KOP_ADXL345();

private:
    // KOP 1-axis gyro
    KopGyro m_gyro;

    // ADXL345 3-axis accelerometer
    I2C* m_accel;

    /* Read raw y-axis from gyro
     * Actually measures y-axis of gyro for consistency with accelerometer
     */
    int readGyroX();

    // Read raw data from accelerometer
    int readAccelX();
    int readAccelY();
    int readAccelZ();

    // Return zero points for gyro
    double getGyroXzero();

    // Return zero points for accelerometer
    double getAccelXzero();
    double getAccelYzero();
    double getAccelZzero();

    // Return sensitivity of gyro in volts per degree/second
    double getGyroLSBsPerUnit();
};

#endif // KOP_ADXL345_HPP
