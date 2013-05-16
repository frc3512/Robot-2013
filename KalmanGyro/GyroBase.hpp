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

/* Usage:
 *
 * This class is an implementation for a gyro-accelerometer Kalman Filter.
 * Since this class can't possibly know how to interface with every gyro and
 * accelerometer that exists, you'll instead handle your gyro/accelerometer in
 * your own derived class and pass in the required data from them via the pure
 * virtual functions.
 *
 * * To use this interface, derive from this class and override all pure
 *   virtual member functions privately.
 *
 * Function clarifications:
 *   * readGyroX() should return the value of the X-axis of the gyro
 *   * readAccelY() should return the value of the Y-axis of the accelerometer
 *   * getGyroZzero() should return the constant representing the zero point of
 *     the Z-axis of the gyro
 *   * getGyroLSBsPerUnit() should return the number of least significant bits
 *     representing a change in the gyro's value of 1 degree/second. This can
 *     be found in the data sheet for the specific gyro you're using.
 *
 * Implementing callCalcAngle():
 *   * When writing the callCalcAngle() function, you only have to call
 *     calcAngle(2)
 *   * Arguments to calcAngle(2):
 *       * 1st: getAccelXangle() or any other axis (like getAccelYangle())
 *       * 2nd: getGyroXrate() or any other axis (like (getGyroYrate())
 *
 * Misc. Help:
 *   * If your accelerometer or gyro doesn't have one or more of the three
 *     axes, create the function in your class and return zero. As long as you
 *     don't use the axis in callCalcAngle(), the value will never be used.
 */

#ifndef GYRO_BASE_HPP
#define GYRO_BASE_HPP

#include <Notifier.h>
#include <Synchronized.h>
#include "FilterBase.hpp"

class GyroBase {
public:
    GyroBase();
    virtual ~GyroBase();

    // Return the internally stored values for gyro angles
    double getXangle();
    double getYangle();
    double getZangle();

    // Used to reset starting angle for the respective axis
    void resetXangle( double newAngle );
    void resetYangle( double newAngle );
    void resetZangle( double newAngle );

    // Return the unbiased rate of the respective axis
    double getXrate();
    double getYrate();
    double getZrate();

protected:
    // Return angle calculated with accelerometer data (returns degrees)
    double getAccelXangle();
    double getAccelYangle();
    double getAccelZangle();

    // Return rate calculated with gyro data
    double getGyroXrate();
    double getGyroYrate();
    double getGyroZrate();

private:
    FilterBase xFilter;
    FilterBase yFilter;
    FilterBase zFilter;

    // Read angle change (usually raw data) from gyro
    virtual int readGyroX() = 0;
    virtual int readGyroY() = 0;
    virtual int readGyroZ() = 0;

    // Read acceleration (usually raw data) from accelerometer
    virtual int readAccelX() = 0;
    virtual int readAccelY() = 0;
    virtual int readAccelZ() = 0;

    // Return zero points for gyro
    virtual double getGyroXzero() = 0;
    virtual double getGyroYzero() = 0;
    virtual double getGyroZzero() = 0;

    // Return zero points for accelerometer
    virtual double getAccelXzero() = 0;
    virtual double getAccelYzero() = 0;
    virtual double getAccelZzero() = 0;

    // Return sensitivity of gyro in least significant bits per degree/second
    virtual double getGyroLSBsPerUnit() = 0;
};

#endif // GYRO_BASE_HPP
