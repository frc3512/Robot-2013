// Copyright (c) FRC Team 3512, Spartatroniks 2013-2017. All Rights Reserved.

#include "ITG3200_ADXL345.hpp"

#include <DigitalModule.h>
#include <I2C.h>

ITG3200_ADXL345::ITG3200_ADXL345(UINT32 slot, UINT32 gyroAddress,
                                 UINT32 accelAddress, UINT32 magnetAddress)
    : m_gyro(NULL), m_accel(NULL), m_magnet(NULL) {
    DigitalModule* module = DigitalModule::GetInstance(slot);

    if (module != NULL) {
        m_gyro = module->GetI2C(gyroAddress);
        m_accel = module->GetI2C(accelAddress);
        m_magnet = module->GetI2C(magnetAddress);
    }

    if (m_gyro != NULL) {
        m_gyro->Write(
            0x16,
            0x1A);  // Sets gyro at +-2000deg/sec  and 98Hz Low pass filter
        m_gyro->Write(0x15, 0x09);  // Sets gyro at 100Hz sample rate
    }

    if (m_accel != NULL) {
        m_accel->Write(0x31, 0x09);  // Full resolution mode
        m_accel->Write(0x2D,
                       0x08);  // Setup ADXL345 for constant measurement mode
    }

    if (m_magnet != NULL) {
        m_magnet->Write(0x08, 0x01);  // ACTIVE mode, RAW data
    }
}

ITG3200_ADXL345::~ITG3200_ADXL345() {
    delete m_gyro;
    delete m_accel;
    delete m_magnet;
}

// Actually measures y-axis of gyro for consistency with accelerometer
int ITG3200_ADXL345::readGyroX() {
    uint8_t data[2];

    if (m_gyro != NULL) {
        m_gyro->Read(0x1F, 2, data);
        return (data[0] << 8) | data[1];
    } else {
        return 0;
    }
}

// Actually measures x-axis of gyro for consistency with accelerometer
int ITG3200_ADXL345::readGyroY() {
    uint8_t data[2];

    if (m_gyro != NULL) {
        m_gyro->Read(0x1D, 2, data);
        return (data[0] << 8) | data[1];
    } else {
        return 0;
    }
}

int ITG3200_ADXL345::readGyroZ() {
    uint8_t data[2];

    if (m_gyro != NULL) {
        m_gyro->Read(0x21, 2, data);
        return (data[0] << 8) | data[1];
    } else {
        return 0;
    }
}

int ITG3200_ADXL345::readAccelX() {
    uint8_t data[2];

    if (m_accel != NULL) {
        m_accel->Read(0x32, 2, data);
        return (data[0] << 8) | data[1];
    } else {
        return 0;
    }
}

int ITG3200_ADXL345::readAccelY() {
    uint8_t data[2];

    if (m_accel != NULL) {
        m_accel->Read(0x34, 2, data);
        return data[0] | (data[1] << 8);
    } else {
        return 0;
    }
}

int ITG3200_ADXL345::readAccelZ() {
    uint8_t data[2];

    if (m_accel != NULL) {
        m_accel->Read(0x36, 2, data);
        return (data[0] << 8) | data[1];
    } else {
        return 0;
    }
}

int ITG3200_ADXL345::readMagnetX() {
    uint8_t data[2];

    if (m_magnet != NULL) {
        m_magnet->Read(0x01, 2, data);

        // TODO Convert from two's complement
        return (data[0] << 8) | data[1];
    } else {
        return 0;
    }
}

int ITG3200_ADXL345::readMagnetY() {
    uint8_t data[2];

    if (m_magnet != NULL) {
        m_magnet->Read(0x03, 2, data);

        // TODO Convert from two's complement
        return (data[0] << 8) | data[1];
    } else {
        return 0;
    }
}

double ITG3200_ADXL345::getGyroXzero() { return 52.3; }

double ITG3200_ADXL345::getGyroYzero() { return -18.5; }

double ITG3200_ADXL345::getGyroZzero() {
    return 0.0;  // TODO Find gyro's real z-axis zero
}

double ITG3200_ADXL345::getAccelXzero() { return -200; }

double ITG3200_ADXL345::getAccelYzero() { return 44; }

double ITG3200_ADXL345::getAccelZzero() { return 660; }

double ITG3200_ADXL345::getMagnetXzero() {
    return 0.0;  // TODO Find magnetometer's real x-axis zero
}

double ITG3200_ADXL345::getMagnetYzero() {
    return 0.0;  // TODO Find magnetometer's real y-axis zero
}

double ITG3200_ADXL345::getGyroLSBsPerUnit() { return 14.375; }
