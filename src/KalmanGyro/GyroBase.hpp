// Copyright (c) 2013-2017 FRC Team 3512. All Rights Reserved.

#pragma once

/* Usage:
 *
 * This class is a wrapper for a gyro-accelerometer Kalman Filter. Since this
 * class can't possibly know how to interface with every gyro and accelerometer
 * that exists, you'll instead handle your gyro/accelerometer in your own
 * derived class and pass in the required data from them via the virtual
 * functions.
 *
 * * To use this interface, derive from this class and override all appropriate
 *   virtual member functions privately.
 *
 * Function clarifications:
 *   * readGyroX() should return the value of the X-axis of the gyro
 *   * readAccelY() should return the value of the Y-axis of the accelerometer
 *   * getGyroZzero() should return the constant representing the zero point of
 *     the Z-axis of the gyro
 *   * getGyroLSBsPerUnit() should return the number of least significant bits
 *     which represent a change in the gyro's value of 1 degree/second. This
 *     can be found in the data sheet for the specific gyro you're using. Some
 *     data sheets may tell you that one LSB is A degrees/second, or A units
 *     per LSB, in which case 1/A, or 1/A LSBs per unit, would be the value to
 *     return from this function.
 *
 * Misc. Help:
 *   * If your accelerometer or gyro doesn't have one or more of the three
 *     axes, don't override the functions related to them. Avoid calling
 *     functions which return data for those unused axes, since the data will
 *     be garbage.
 */

#include "GyroFilter.hpp"

class GyroBase {
public:
    GyroBase();
    virtual ~GyroBase();

    // Return the internally stored values for gyro angles
    double getXangle();
    double getYangle();
    double getZangle();

    // Used to reset starting angle for the respective axis
    void resetXangle(double newAngle);
    void resetYangle(double newAngle);
    void resetZangle(double newAngle);

    // Return the unbiased rate of the respective axis
    double getXrate();
    double getYrate();
    double getZrate();

protected:
    // Return rate calculated with gyro data
    double getGyroXrate();
    double getGyroYrate();
    double getGyroZrate();

    // Return angle calculated with accelerometer or magnetometer data (returns
    // degrees)
    double getAccelXangle();
    double getAccelYangle();
    double getMagnetZangle();

private:
    GyroFilter xFilter;
    GyroFilter yFilter;
    GyroFilter zFilter;

    // Read angle change (usually raw data) from gyro
    virtual int readGyroX();
    virtual int readGyroY();
    virtual int readGyroZ();

    // Read acceleration (usually raw data) from accelerometer
    virtual int readAccelX();
    virtual int readAccelY();
    virtual int readAccelZ();

    // Read magnetic field strength (usually raw data) from magnetometer
    virtual int readMagnetX();
    virtual int readMagnetY();

    // Return zero points for gyro
    virtual double getGyroXzero();
    virtual double getGyroYzero();
    virtual double getGyroZzero();

    // Return zero points for accelerometer
    virtual double getAccelXzero();
    virtual double getAccelYzero();
    virtual double getAccelZzero();

    // Return zero points for magnetometer
    virtual double getMagnetXzero();
    virtual double getMagnetYzero();

    /* Return sensitivity of gyro in least significant bits per degree/second.
     * If you're using an analog gyro, LSBs would be volts instead, but the
     * underlying algorithm is the same.
     */
    virtual double getGyroLSBsPerUnit() = 0;
};
