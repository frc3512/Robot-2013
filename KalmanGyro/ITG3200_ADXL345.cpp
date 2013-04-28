//=============================================================================
//File Name: ITG3200_ADXL345.cpp
//Description: Implements a Kalman filter with the I2C IMU with the ITG3200
//             3-axis gyro and the ADXL345 3-axis accelerometer
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "ITG3200_ADXL345.hpp"

#include <DigitalModule.h>
#include <I2C.h>

// Found by experimenting
double gZeroValue[5] = {
        -200, // Accelerometer X
        44,   // Accelerometer Y
        660,  // Accelerometer Z
        52.3, // Gyro X
        -18.5 // Gyro Y
};

const double gGyroLSBWeight = 14.375;

ITG3200_ADXL345::ITG3200_ADXL345( UINT32 slot , UINT32 gyroAddress , UINT32 accelAddress )
: m_gyro( NULL ) , m_accel( NULL ) {
    DigitalModule* module = DigitalModule::GetInstance( slot );

    if ( module != NULL ) {
        m_gyro = module->GetI2C( gyroAddress );
        m_accel = module->GetI2C( accelAddress );
    }

    if ( m_gyro != NULL ) {
        m_gyro->Write( 0x16 , 0x1A ); // Sets gyro at +-2000deg/sec  and 98Hz Low pass filter
        m_gyro->Write( 0x15 , 0x09 ); // Sets gyro at 100Hz sample rate
    }

    if ( m_accel != NULL ) {
        m_accel->Write( 0x31 , 0x09 ); // Full resolution mode
        m_accel->Write( 0x2D , 0x08 ); // Setup ADXL345 for constant measurement mode
    }
}

ITG3200_ADXL345::~ITG3200_ADXL345() {
    delete m_gyro;
    delete m_accel;
}

// Actually measures y-axis of gyro for consistency with accelerometer
int ITG3200_ADXL345::readGyroX() {
    uint8_t* data = NULL;

    m_gyro->Read( 0x1F , 2 , data );

    return (data[0] << 8) | data[1];
}

// Actually measures x-axis of gyro for consistency with accelerometer
int ITG3200_ADXL345::readGyroY() {
    uint8_t* data = NULL;

    m_gyro->Read( 0x1D , 2 , data );

    return (data[0] << 8) | data[1];
}

int ITG3200_ADXL345::readGyroZ() {
    uint8_t* data = NULL;

    m_gyro->Read( 0x21 , 2 , data );

    return (data[0] << 8) | data[1];
}

int ITG3200_ADXL345::readAccelX() {
    uint8_t* data = NULL;

    m_accel->Read( 0x32 , 2 , data );

    return data[0] | (data[1] << 8);
}

int ITG3200_ADXL345::readAccelY() {
    uint8_t* data = NULL;

    m_accel->Read( 0x34 , 2 , data );

    return data[0] | (data[1] << 8);
}

int ITG3200_ADXL345::readAccelZ() {
    uint8_t* data = NULL;

    m_accel->Read( 0x36 , 2 , data );

    return data[0] | (data[1] << 8);
}

double ITG3200_ADXL345::getAccelXzero() {
    return gZeroValue[0];
}

double ITG3200_ADXL345::getAccelYzero() {
    return gZeroValue[1];
}

double ITG3200_ADXL345::getAccelZzero() {
    return gZeroValue[2];
}

double ITG3200_ADXL345::getGyroXzero() {
    return gZeroValue[3];
}

double ITG3200_ADXL345::getGyroYzero() {
    return gZeroValue[4];
}

double ITG3200_ADXL345::getGyroLSBWeight() {
    return gGyroLSBWeight;
}

void ITG3200_ADXL345::callCalcAngle( double dt ) {
    calcAngle( getAccelXangle() , getGyroXrate() , dt );
}
