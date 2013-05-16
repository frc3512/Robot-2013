/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <cmath>
#include <Timer.h>
#include "GyroBase.hpp"
#include "FilterBase.hpp"

GyroBase::GyroBase() :
xFilter( &GyroBase::getAccelXangle , &GyroBase::getGyroXrate , this ) ,
yFilter( &GyroBase::getAccelYangle , &GyroBase::getGyroYrate , this ) ,
zFilter( &GyroBase::getAccelZangle , &GyroBase::getGyroZrate , this )
{
}

GyroBase::~GyroBase() {
}

double GyroBase::getXangle() {
    return xFilter.getAngle();
}

double GyroBase::getYangle() {
    return yFilter.getAngle();
}

double GyroBase::getZangle() {
    return zFilter.getAngle();
}

void GyroBase::resetXangle( double newAngle ) {
    return xFilter.resetAngle( newAngle );
}

void GyroBase::resetYangle( double newAngle ) {
    return yFilter.resetAngle( newAngle );
}

void GyroBase::resetZangle( double newAngle ) {
    return zFilter.resetAngle( newAngle );
}

double GyroBase::getXrate() {
    return xFilter.getRate();
}

double GyroBase::getYrate() {
    return yFilter.getRate();
}

double GyroBase::getZrate() {
    return zFilter.getRate();
}

double GyroBase::getAccelXangle() {
    double accelXval = static_cast<double>(readAccelX()) - getAccelXzero();
    double accelZval = static_cast<double>(readAccelZ()) - getAccelZzero();

    // ( std::atan2( accelXval , accelZval ) + 3.14159265 ) * 180.0 / 3.14159265;
    return 180.0 * std::atan2( accelXval , accelZval ) / 3.14159265 + 180.0;
}

double GyroBase::getAccelYangle() {
    double accelYval = static_cast<double>(readAccelY()) - getAccelYzero();
    double accelZval = static_cast<double>(readAccelZ()) - getAccelZzero();

    // ( std::atan2( accelYval , accelZval ) + 3.14159265 ) * 180.0 / 3.14159265
    return 180.0 * std::atan2( accelYval , accelZval ) / 3.14159265 + 180.0;
}

double GyroBase::getAccelZangle() { // TODO: correct order of atan2 args?
    double accelXval = static_cast<double>(readAccelX()) - getAccelXzero();
    double accelYval = static_cast<double>(readAccelY()) - getAccelYzero();

    // ( std::atan2( accelXval , accelYval ) + 3.14159265 ) * 180.0 / 3.14159265
    return 180.0 * std::atan2( accelXval , accelYval ) / 3.14159265 + 180.0;
}

double GyroBase::getGyroXrate() {
    return -(static_cast<double>(readGyroX()) - getGyroXzero()) / getGyroLSBsPerUnit();
}

double GyroBase::getGyroYrate() {
    return (static_cast<double>(readGyroY()) - getGyroYzero()) / getGyroLSBsPerUnit();
}

double GyroBase::getGyroZrate() { // TODO: negate return value?
    return (static_cast<double>(readGyroZ()) - getGyroZzero()) / getGyroLSBsPerUnit();
}
