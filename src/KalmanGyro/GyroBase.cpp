// Copyright (c) FRC Team 3512, Spartatroniks 2013-2017. All Rights Reserved.

#include "GyroBase.hpp"

#include <cmath>

#include <Timer.h>

GyroBase::GyroBase()
    : xFilter(&GyroBase::getAccelXangle, &GyroBase::getGyroXrate, this),
      yFilter(&GyroBase::getAccelYangle, &GyroBase::getGyroYrate, this),
      zFilter(&GyroBase::getMagnetZangle, &GyroBase::getGyroZrate, this) {
    xFilter.setQAngle(0.001);
    xFilter.setQBias(0.003);
    xFilter.setRMeasure(0.03);

    yFilter.setQAngle(0.001);
    yFilter.setQBias(0.003);
    yFilter.setRMeasure(0.03);

    zFilter.setQAngle(0.001);
    zFilter.setQBias(0.003);
    zFilter.setRMeasure(0.03);
}

GyroBase::~GyroBase() {}

double GyroBase::getXangle() { return xFilter.getAngle(); }

double GyroBase::getYangle() { return yFilter.getAngle(); }

double GyroBase::getZangle() { return zFilter.getAngle(); }

void GyroBase::resetXangle(double newAngle) {
    return xFilter.resetAngle(newAngle);
}

void GyroBase::resetYangle(double newAngle) {
    return yFilter.resetAngle(newAngle);
}

void GyroBase::resetZangle(double newAngle) {
    return zFilter.resetAngle(newAngle);
}

double GyroBase::getXrate() { return xFilter.getRate(); }

double GyroBase::getYrate() { return yFilter.getRate(); }

double GyroBase::getZrate() { return zFilter.getRate(); }

double GyroBase::getAccelXangle() {
    double accelXval = static_cast<double>(readAccelX()) - getAccelXzero();
    double accelZval = static_cast<double>(readAccelZ()) - getAccelZzero();

    // ( std::atan2( accelXval , accelZval ) + 3.14159265 ) * 180.0 /
    // 3.14159265;
    return 180.0 * std::atan2(accelXval, accelZval) / 3.14159265 + 180.0;
}

double GyroBase::getAccelYangle() {
    double accelYval = static_cast<double>(readAccelY()) - getAccelYzero();
    double accelZval = static_cast<double>(readAccelZ()) - getAccelZzero();

    // ( std::atan2( accelYval , accelZval ) + 3.14159265 ) * 180.0 / 3.14159265
    return 180.0 * std::atan2(accelYval, accelZval) / 3.14159265 + 180.0;
}

double GyroBase::getMagnetZangle() {  // TODO: correct order of atan2 args?
    double magnetXval = static_cast<double>(readMagnetX()) - getMagnetXzero();
    double magnetYval = static_cast<double>(readMagnetY()) - getMagnetYzero();

    // ( std::atan2( accelXval , accelYval ) + 3.14159265 ) * 180.0 / 3.14159265
    return 180.0 * std::atan2(magnetXval, magnetYval) / 3.14159265 + 180.0;
}

double GyroBase::getGyroXrate() {
    return -(static_cast<double>(readGyroX()) - getGyroXzero()) /
           getGyroLSBsPerUnit();
}

double GyroBase::getGyroYrate() {
    return (static_cast<double>(readGyroY()) - getGyroYzero()) /
           getGyroLSBsPerUnit();
}

double GyroBase::getGyroZrate() {  // TODO: negate return value?
    return (static_cast<double>(readGyroZ()) - getGyroZzero()) /
           getGyroLSBsPerUnit();
}

int GyroBase::readGyroX() { return 0; }

int GyroBase::readGyroY() { return 0; }

int GyroBase::readGyroZ() { return 0; }

int GyroBase::readAccelX() { return 0; }

int GyroBase::readAccelY() { return 0; }

int GyroBase::readAccelZ() { return 0; }

int GyroBase::readMagnetX() { return 0; }

int GyroBase::readMagnetY() { return 0; }

double GyroBase::getGyroXzero() { return 0.0; }

double GyroBase::getGyroYzero() { return 0.0; }

double GyroBase::getGyroZzero() { return 0.0; }

double GyroBase::getAccelXzero() { return 0.0; }

double GyroBase::getAccelYzero() { return 0.0; }

double GyroBase::getAccelZzero() { return 0.0; }

double GyroBase::getMagnetXzero() { return 0.0; }

double GyroBase::getMagnetYzero() { return 0.0; }
