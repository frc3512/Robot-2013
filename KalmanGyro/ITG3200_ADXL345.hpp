//=============================================================================
//File Name: ITG3200_ADXL345.hpp
//Description: Implements a Kalman filter with the I2C IMU with the ITG3200
//             3-axis gyro and the ADXL345 3-axis accelerometer
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#ifndef ITG3200_ADXL345_HPP
#define ITG3200_ADXL345_HPP

#include "KalmanGyro.hpp"

#include <SensorBase.h>
#include <DigitalModule.h>

class I2C;

class ITG3200_ADXL345 : public KalmanGyro , public SensorBase {
public:
    ITG3200_ADXL345( UINT32 slot , UINT32 gyroAddress , UINT32 accelAddress );
    virtual ~ITG3200_ADXL345();

private:
    // ITG3200 3-axis gyro
    I2C* m_gyro;

    // ADXL345 3-axis accelerometer
    I2C* m_accel;

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

    // Return zero points for gyro
    double getGyroXzero();
    double getGyroYzero();
    double getGyroZzero();

    // Return zero points for accelerometer
    double getAccelXzero();
    double getAccelYzero();
    double getAccelZzero();

    // Return sensitivity of gyro in least significant bits per degree/second
    double getGyroLSBsPerUnit();

    void callCalcAngle();
};

#endif // ITG3200_ADXL345_HPP
