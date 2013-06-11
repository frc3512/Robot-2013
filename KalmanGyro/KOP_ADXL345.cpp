//=============================================================================
//File Name: KOP_ADXL345.cpp
//Description: Implements a Kalman filter with the KOP 1-axis gyro and the
//             ADXL345 3-axis accelerometer
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "KOP_ADXL345.hpp"

#include <DigitalModule.h>
#include <I2C.h>

KOP_ADXL345::KOP_ADXL345( UINT32 gyroSlot , UINT32 gyroChannel , UINT32 accelSlot , UINT32 accelAddress )
: m_gyro( gyroSlot , gyroChannel ) , m_accel( NULL ) {
    DigitalModule* accelModule = DigitalModule::GetInstance( accelSlot );

    if ( accelModule != NULL ) {
        m_accel = accelModule->GetI2C( accelAddress );
    }

    if ( m_accel != NULL ) {
        m_accel->Write( 0x31 , 0x09 ); // Full resolution mode
        m_accel->Write( 0x2D , 0x08 ); // Setup ADXL345 for constant measurement mode
    }
}

KOP_ADXL345::~KOP_ADXL345() {
    delete m_accel;
}

// Actually measures y-axis of gyro for consistency with accelerometer
int KOP_ADXL345::readGyroX() {
    return (m_gyro.GetVoltage() - getGyroXzero()) / getGyroLSBsPerUnit();
}

int KOP_ADXL345::readAccelX() {
    uint8_t data[2];

    if ( m_accel != NULL ) {
        m_accel->Read( 0x32 , 2 , data );
        return (data[0] << 8) | data[1];
    }
    else {
        return 0;
    }
}

int KOP_ADXL345::readAccelY() {
    uint8_t data[2];

    if ( m_accel != NULL ) {
        m_accel->Read( 0x34 , 2 , data );
        return data[0] | (data[1] << 8);
    }
    else {
        return 0;
    }
}

int KOP_ADXL345::readAccelZ() {
    uint8_t data[2];

    if ( m_accel != NULL ) {
        m_accel->Read( 0x36 , 2 , data );
        return (data[0] << 8) | data[1];
    }
    else {
        return 0;
    }
}

double KOP_ADXL345::getGyroXzero() {
    return 2.5;
}

double KOP_ADXL345::getAccelXzero() {
    return -200;
}

double KOP_ADXL345::getAccelYzero() {
    return 44;
}

double KOP_ADXL345::getAccelZzero() {
    return 660;
}

double KOP_ADXL345::getGyroLSBsPerUnit() {
    return 0.007;
}
