//=============================================================================
//File Name: KOP_ADXL345.cpp
//Description: Implements a Kalman filter with the KOP 1-axis gyro and the
//             ADXL345 3-axis accelerometer
//Author: FRC Team 3512, Spartatroniks
//=============================================================================

#include "KOP_ADXL345.hpp"

#include <AnalogModule.h>
#include <DigitalModule.h>
#include <I2C.h>

KOP_ADXL345::KOP_ADXL345( UINT32 gyroSlot , UINT32 gyroChannel , UINT32 i2cSlot , UINT32 accelAddress , UINT32 magnetAddress )
: m_gyro( NULL ) , m_accel( NULL ) , m_magnet( NULL ) {
    m_gyro = AnalogModule::GetInstance( gyroSlot );
    m_gyroChannel = gyroChannel;

    DigitalModule* i2cModule = DigitalModule::GetInstance( i2cSlot );

    if ( i2cModule != NULL ) {
        m_accel = i2cModule->GetI2C( accelAddress );
        m_magnet = i2cModule->GetI2C( magnetAddress );
    }

    if ( m_accel != NULL ) {
        m_accel->Write( 0x31 , 0x09 ); // Full resolution mode
        m_accel->Write( 0x2D , 0x08 ); // Setup ADXL345 for constant measurement mode
    }

    if ( m_magnet != NULL ) {
        m_magnet->Write( 0x08 , 0x01 ); // ACTIVE mode, RAW data
    }
}

KOP_ADXL345::~KOP_ADXL345() {
    delete m_accel;
    delete m_magnet;
}

// Actually measures y-axis of gyro for consistency with accelerometer
int KOP_ADXL345::readGyroX() {
    if ( m_gyro != NULL ) {
        return static_cast<int>((m_gyro->GetAverageVoltage( m_gyroChannel ) - getGyroXzero()) / getGyroLSBsPerUnit());
    }
    else {
        return 0;
    }
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

int KOP_ADXL345::readMagnetX() {
    uint8_t data[2];

    if ( m_magnet != NULL ) {
        m_magnet->Read( 0x01 , 2 , data );

        // TODO Convert from two's complement
        return (data[0] << 8) | data[1];
    }
    else {
        return 0;
    }
}

int KOP_ADXL345::readMagnetY() {
    uint8_t data[2];

    if ( m_magnet != NULL ) {
        m_magnet->Read( 0x03 , 2 , data );

        // TODO Convert from two's complement
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

double KOP_ADXL345::getMagnetXzero() {
    return 0.0; // TODO Find magnetometer's real x-axis zero
}

double KOP_ADXL345::getMagnetYzero() {
    return 0.0; // TODO Find magnetometer's real y-axis zero
}

double KOP_ADXL345::getGyroLSBsPerUnit() {
    return 0.007;
}
